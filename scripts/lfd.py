#%%
#!/usr/bin/env python
import cv2
import rospy
import math
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import tf
from camera_feedback import CameraFeedback, image_process
from std_msgs.msg import Float32
import rospkg
from geometry_msgs.msg import PoseStamped, Pose
from panda_ros import Panda
from feedback import Feedback
from transfom import Transform 
from panda_ros.pose_transform_functions import position_2_array, array_quat_2_pose, list_2_quaternion
from copy import deepcopy
import pickle
class LfD(Feedback):
    def __init__(self):
        rospy.init_node("learning_node")
        super(LfD, self).__init__()
        self.control_rate = 30
        self.rate=rospy.Rate(self.control_rate)
        
        self._tf_broadcaster = tf.TransformBroadcaster()

        self.camera_feedback = CameraFeedback()
        self.robot= Panda()
        self.curr_image = None

        self.end = False
        self.grip_open_width = 0.02
        self.grip_close_width = 0.0
        self.retry_limit = 3


        self.insertion_force_threshold = 5
        self.retry_counter = 0

        self.servoing_transform = np.eye(4)

        rospy.sleep(1)

    def kinesthetic_teaching(self, trigger=0.005):
        init_pos = self.robot.curr_pos
        perturbation = 0 
        print("Move robot to start recording.")
        while perturbation < trigger:
            perturbation = math.sqrt((self.robot.curr_pos[0]-init_pos[0])**2 + (self.robot.curr_pos[1]-init_pos[1])**2 + (self.robot.curr_pos[2]-init_pos[2])**2)
                # trigger for starting the recording
        self.robot.set_stiffness(0, 0, 0, 0, 0, 0, 0)
        self.recorded_pose = []
        self.recorded_gripper = []
        self.recorded_img = []
        self.recorded_img_feedback_flag = []
        self.recorded_spiral_flag = []
        if self.gripper_width < self.grip_open_width * 0.9:
            self.grip_value = 0
        else:
            self.grip_value = self.grip_open_width

        print("Recording started. Press Esc to stop.")
        while not self.end:
            while(self.pause):
                self.rate.sleep()
            self.recorded_pose.append(self.robot.curr_pose)               
            self.recorded_gripper.append(self.grip_value)

            resized_img_gray=image_process(self.curr_image, self.ds_factor, self.row_crop_pct_top , self.row_crop_pct_bot,self.col_crop_pct_left, self.col_crop_pct_right)
            resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img_gray)
            self.cropped_img_pub.publish(resized_img_msg)
            self.recorded_img.append(resized_img_gray.reshape((1, resized_img_gray.shape[0], resized_img_gray.shape[1])))
            self.recorded_img_feedback_flag.append(self.img_feedback_flag)
            self.recorded_spiral_flag.append(self.spiral_flag)

            self.rate.sleep()

        goal = self.curr_pose
        goal.header = {"seq": 1, "stamp": rospy.Time.now(), "frame_id": "map"}
        self.robot.goal_pub.publish(goal)
        self.robot.set_stiffness(3000, 3000, 3000, 40, 40, 40, 0)
        rospy.loginfo("Ending trajectory recording")

    def execute(self, retry_insertion_flag=0, execution_speed=1):
        self.rate=rospy.Rate(self.control_rate * execution_speed)

        total_transform = self.localization_transform
        self.go_to_pose(transform_pose(self.recorded_pose[0],total_transform)) 

        self.time_index=0

        self.activate_gripper(self.recorded_gripper[self.time_index])


        while self.time_index < self.recorded_traj.shape[1]-1:

            self.human_feedback()

            if self.recorded_img_feedback_flag[self.time_index]:
                self.servoing_transform=self.camera_feedback.sift_matching(time_index=self.time_index) @ self.servoing_transform
                total_transform = self.servoing_transform @ total_transform

            if self.recorded_spiral_flag[self.time_index]:
                if self.force.z > self.insertion_force_threshold:
                    transform_spiral = self.spiral_search(goal_pose)
                    total_transform = transform_spiral @ total_transform


            force = np.linalg.norm([self.force.x, self.force.y, self.force.z])
            if retry_insertion_flag and force > self.insertion_force_threshold:
                if self.retry_counter >= self.retry_limit:
                    self.move_gripper(self.grip_open_width)
                    break
                self.go_to_pose(start) # this whould not be start since start does not have the corrections applied to it. 
                self.time_index = 0
                self.retry_counter = self.retry_counter + 1

            if self.safety_check:
                    self.time_index += 1
                    self.robot.set_stiffness(self.K_pos, self.K_pos, self.K_pos, 0, 0, 0, self.K_ns)
                    self.set_K.update_configuration({"max_delta_lin": 0.2})
                    self.set_K.update_configuration({"max_delta_ori": 0.5}) 
            else:
                self.robot.set_stiffness(self.K_pos_safe, self.K_pos_safe, self.K_pos_safe, 0, 0, 0, 5)
                self.set_K.update_configuration({"max_delta_lin": 0.05})
                self.set_K.update_configuration({"max_delta_ori": 0.1})   

            ### Publish the goal
            goal_pose = self.recorded_pose[self.time_index]
            goal_pose.header = {"seq": 1, "stamp": rospy.Time.now(), "frame_id": "map"}

            goal_pose=self.transform_pose(goal_pose, total_transform)
            self.activate_gripper(self.recorded_gripper[self.time_index])
            self.robot.goal_pub.publish(goal) 
            self.rate.sleep()

    def activate_gripper(self, grip_value):
        if grip_value < self.grip_open_width * 0.9:
            self.robot.grasp_gripper(grip_value)
            self.gripper_opened = False
            time.sleep(0.1)
        esle: 
            self.robot.move_gripper(grip_value)     
            self.gripper_opened = True              
            time.sleep(0.1)

    def spiral_search(self, goal_pose: PoseStamped, force_min_exit=1): 
        # force_min_exit in Newton. If the verical force is lower than this value, the spiral search is considered successful
        max_spiral_time = 30 # seconds
        increase_radius_per_second = 0.0005 # meters, distance from center of the spiral after 1 second
        rounds_per_second = 1 # how many rounds does the spiral every second
        dt = 1. / control_rate
        
        # goal_pose = array_quat_2_pose(goal_init, ori_quat)
        goal_start = deepcopy(goal_pose)
        goal_final = deepcopy(goal_pose)
        time= 0 
        self.robot.set_stiffness(4000, 4000, 1000, 30, 30, 30, 0) # get more compliant in z direction
        for _ in range(max_spiral_time * control_rate):   
            goal_final.pose.position.x = goal_start.pose.position.x + np.cos(
                2 * np.pi *rounds_per_second*time) * increase_radius_per_second * time
            goal_final.pose.position.y = goal_start.pose.position.y + np.sin(
                2 * np.pi *rounds_per_second* time) * increase_radius_per_second * time
            self.robot.goal_pub.publish(goal_final)
            if self.force.z <= force_min_exit: 
                break
            time += dt
            self.rate.sleep()
        self.robot.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)    
        trasform = transform_between_poses(goal_final, goal_start ) 

        return trasform



def save(self, file='last'):
    ros_pack = rospkg.RosPack()
    self._package_path = ros_pack.get_path('trajectory_data')
    with open(self._package_path + '/trajectories/' + str(file) + '.pkl', 'wb') as f:
        pickle.dump({
            'pose': self.recorded_pose,
            'grip': self.recorded_gripper,
            'img': self.recorded_img,
            'img_feedback_flag': self.recorded_img_feedback_flag,
            'spiral_flag': self.recorded_spiral_flag
        }, f)
    
def load(self, file='last'):
    ros_pack = rospkg.RosPack()
    self._package_path = ros_pack.get_path('trajectory_data')
    with open(self._package_path + '/trajectories/' + str(file) + '.pkl', 'rb') as f:
        data = pickle.load(f)
    self.recorded_pose = data['pose']
    self.recorded_gripper = data['grip']
    self.recorded_img = data['img']
    self.recorded_img_feedback_flag = data['img_feedback_flag']
    self.recorded_spiral_flag = data['spiral_flag']
