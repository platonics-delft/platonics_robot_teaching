import rospy
import math
import numpy as np
import time
import tf
from camera_feedback import Camera
from std_msgs.msg import Header
import rospkg
from geometry_msgs.msg import PoseStamped
from panda_ros import Panda
from feedback_franka_buttons import FeedbackButtons as Feedback
# from feedback_keyboard import FeedbackKeyboard as Feedback
from panda_ros.pose_transform_functions import transform_pose, transform_between_poses, interpolate_poses
from copy import deepcopy
import pickle



class LfD():
    data = {}
    def __init__(self):
        super(LfD, self).__init__()
        self.control_rate = 30
        self.rate=rospy.Rate(self.control_rate)
        
        self._tf_broadcaster = tf.TransformBroadcaster()

        self.camera = Camera()
        self.robot= Panda()
        self.buttons = Feedback()

        self.buttons.end = False
        self.grip_open_width = 0.02
        self.grip_close_width = 0.0
        self.retry_limit = 3


        self.insertion_force_threshold = 5
        self.retry_counter = 0

        self.localization_transform = np.eye(4)

        self.safe_distance_lin=0.005
        self.safe_distance_ori=0.005

        rospy.sleep(1)

    def kinesthetic_teaching(self, trigger=0.005):
        init_pos = self.robot.curr_pos
        perturbation = 0 
        print("Move robot to start recording.")
        while perturbation < trigger:
            perturbation = math.sqrt((self.robot.curr_pos[0]-init_pos[0])**2 + (self.robot.curr_pos[1]-init_pos[1])**2 + (self.robot.curr_pos[2]-init_pos[2])**2)
                # trigger for starting the recording
        self.robot.set_stiffness(0, 0, 0, 0, 0, 0, 0)
        self.recorded_pose = [self.robot.curr_pose]
        self.recorded_gripper =  [self.grip_close_width if self.buttons.gripper_closed else self.grip_open_width]
        self.recorded_img = [self.camera.curr_image]
        self.recorded_img_feedback_flag = [0]
        self.recorded_spiral_flag = [0]
        if self.robot.gripper_width < self.grip_open_width * 0.9:
            self.buttons.gripper_closed = True
            self.gripper_state = True
        else:
            self.buttons.gripper_closed = False
            self.gripper_state = False

        print("Recording started. Press Esc to stop.")

        while not self.buttons.end:
            while(self.buttons.pause):
                self.rate.sleep()
            poses=  interpolate_poses(self.recorded_pose[-1],self.robot.curr_pose, self.safe_distance_lin, self.safe_distance_ori)[1:]
            
            grip_value = self.grip_close_width if self.buttons.gripper_closed else self.grip_open_width
            change_gripper_state = self.gripper_state != self.buttons.gripper_closed
            self.gripper_state = self.buttons.gripper_closed
            # print("Change gripper state: ", change_gripper_state)
            if change_gripper_state:
                self.activate_gripper(grip_value)
            

            if self.buttons.stiff_rotation:
                self.robot.goal_pub.publish(self.robot.curr_pose)
                self.robot.set_stiffness(0, 0,0, 30, 30, 30, 0)
            else:
                self.robot.set_stiffness(0, 0, 0, 0, 0, 0, 0)

            self.recorded_pose.extend(poses)
            self.recorded_gripper.extend([grip_value]*len(poses))
            self.recorded_img.extend([self.camera.curr_image]*len(poses))
            self.recorded_img_feedback_flag.extend([self.buttons.img_feedback_flag]*len(poses))
            self.recorded_spiral_flag.extend([self.buttons.spiral_flag]*len(poses))
            self.rate.sleep()

        goal = self.robot.curr_pose
        goal.header = Header(seq=1, stamp=rospy.Time.now(), frame_id="map")
        self.robot.goal_pub.publish(goal)
        self.robot.set_stiffness(3000, 3000, 3000, 40, 40, 40, 0)
        rospy.loginfo("Ending trajectory recording")
        self.data = {
            'recorded_pose': self.recorded_pose,
            'recorded_gripper': self.recorded_gripper,
            'recorded_img': self.recorded_img,
            'recorded_img_feedback_flag': self.recorded_img_feedback_flag,
            'recorded_spiral_flag': self.recorded_spiral_flag
        }


    def execute(self, retry_insertion_flag=0, execution_speed=1):

        self.robot.set_stiffness(3000, 3000, 3000, 30, 30, 30, 0)
        self.robot.set_K.update_configuration({"max_delta_lin": 0.05})
        self.robot.set_K.update_configuration({"max_delta_ori": 0.15}) 
        self.rate=rospy.Rate(self.control_rate * execution_speed)

        total_transform = self.localization_transform
        self.servoing_transform = np.eye(4)
        self.spiral_transform = np.eye(4)
        self.robot.go_to_pose(transform_pose(self.data['recorded_pose'][0],total_transform)) 

        self.time_index=0

        self.activate_gripper(self.data['recorded_gripper'][self.time_index])
        if self.robot.gripper_width < self.grip_open_width * 0.9:
            self.buttons.gripper_closed = True
            self.gripper_state = True
        else:
            self.buttons.gripper_closed = False
            self.gripper_state = False

        self.robot.change_in_safety_check = False
        while not(self.buttons.end):

            self.data= self.buttons.human_feedback(self.data, self.time_index)

            ### Perform camera corrections
            if self.data['recorded_img_feedback_flag'][self.time_index]:
                self.servoing_transform=self.camera.sift_matching(target_img=self.data['recorded_img'][self.time_index])
                total_transform = self.servoing_transform @ total_transform
            
            ### Perform spiral search
            if self.data['recorded_spiral_flag'][self.time_index]:
                if self.robot.force.z > self.insertion_force_threshold:
                    self.spiral_transform = self.spiral_search(goal_pose)
                    total_transform = self.spiral_transform @ total_transform

            ### Retry check
            force = np.linalg.norm([self.robot.force.x, self.robot.force.y, self.robot.force.z])
            if retry_insertion_flag and force > self.insertion_force_threshold:
                if self.retry_counter >= self.retry_limit:
                    self.move_gripper(self.grip_open_width)
                    break
                self.robot.go_to_pose(transform_pose(self.data['recorded_pose'][0],total_transform)) 
                self.time_index = 0
                self.retry_counter = self.retry_counter + 1
            ### Safety check
            if self.robot.safety_check: self.time_index += 1
            if self.robot.change_in_safety_check:
                if self.robot.safety_check:
                        self.robot.set_stiffness(self.robot.K_pos, self.robot.K_pos, self.robot.K_pos, self.robot.K_ori, self.robot.K_ori, self.robot.K_ori, 0)
                else:
                    # print("Safety violation detected. Making the robot compliant")
                    self.robot.set_stiffness(self.robot.K_pos_safe, self.robot.K_pos_safe, self.robot.K_pos_safe, self.robot.K_ori_safe, self.robot.K_ori_safe, self.robot.K_ori_safe, 0)
            ### Publish the goal pose
            goal_pose = self.data['recorded_pose'][self.time_index]
            goal_pose.header = {"seq": 1, "stamp": rospy.Time.now(), "frame_id": "map"}

            goal_pose=transform_pose(goal_pose, total_transform)
            self.robot.goal_pub.publish(goal_pose) 

            # Activate the gripper
            if self.data['recorded_gripper'][self.time_index] < self.grip_open_width * 0.9:
                self.buttons.gripper_closed = True
            else:
                self.buttons.gripper_closed = False
            change_gripper_state = self.gripper_state != self.buttons.gripper_closed
            self.gripper_state = self.buttons.gripper_closed
            if change_gripper_state:
                self.activate_gripper(self.data['recorded_gripper'][self.time_index])

            if self.time_index == (len(self.data['recorded_pose'])-1):
                break

            self.rate.sleep()

    def activate_gripper(self, grip_value):
        if grip_value < self.grip_open_width * 0.9:
            self.robot.grasp_gripper(grip_value)
            time.sleep(0.1)
        else: 
            self.robot.move_gripper(grip_value)     
            time.sleep(0.1)

    def spiral_search(self, goal_pose: PoseStamped, force_min_exit=1): 
        # force_min_exit in Newton. If the verical force is lower than this value, the spiral search is considered successful
        max_spiral_time = 30 # seconds
        increase_radius_per_second = 0.0005 # meters, distance from center of the spiral after 1 second
        rounds_per_second = 1 # how many rounds does the spiral every second
        dt = 1. / self.control_rate

        goal_start = deepcopy(goal_pose)
        goal_final = deepcopy(goal_pose)
        time= 0 
        self.robot.set_stiffness(4000, 4000, 1000, 30, 30, 30, 0) # get more compliant in z direction
        for _ in range(max_spiral_time * self.control_rate):   
            goal_final.pose.position.x = goal_start.pose.position.x + np.cos(
                2 * np.pi *rounds_per_second*time) * increase_radius_per_second * time
            goal_final.pose.position.y = goal_start.pose.position.y + np.sin(
                2 * np.pi *rounds_per_second* time) * increase_radius_per_second * time
            self.robot.goal_pub.publish(goal_final)
            if self.robot.force.z <= force_min_exit: 
                break
            time += dt
            self.rate.sleep()
        self.robot.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)    
        trasform = transform_between_poses(goal_final, goal_start ) 

        return trasform



    def save(self, file='last'):
        print("Saving trajectory")
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_data')
        with open(self._package_path + '/trajectories/' + str(file) + '.pkl', 'wb') as f:
            pickle.dump(self.data, f)
        print("Trajectory saved")
        
    def load(self, file='last'):
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_data')
        with open(self._package_path + '/trajectories/' + str(file) + '.pkl', 'rb') as f:
            data = pickle.load(f)
        self.data = data

