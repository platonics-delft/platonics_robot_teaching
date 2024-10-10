import rospy
from typing import List
import os
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
from feedback_spacenav import FeedbackSpacenav
# from feedback_keyboard import FeedbackKeyboard as Feedback
from panda_ros.pose_transform_functions import transform_pose, transform_between_poses, interpolate_poses, transformation_2_pose
from copy import deepcopy
import pickle
from geometry_msgs.msg import Point

def point_iter(self):
    yield self.x
    yield self.y
    yield self.z

Point.__iter__ = point_iter

SPEED_FACTOR = 3



class LfD():
    data = {}
    def __init__(self):
        super(LfD, self).__init__()
        self.control_rate = 60
        self.rate=rospy.Rate(self.control_rate)
        
        self._tf_broadcaster = tf.TransformBroadcaster()

        self.camera = Camera()
        self.robot= Panda()
        self.robot.attractor_distance_threshold=0.01
        self.buttons = Feedback()
        self.space_nav_feedback = FeedbackSpacenav()

        self.buttons.end = False
        self.grip_open_width = 0.02
        self.grip_close_width = 0.0
        self.retry_limit = 3
        self.compensation_rate = 5
        self.window_size_seconds = 1
        self.window_size_steps = self.window_size_seconds * self.control_rate


        self.insertion_force_threshold = 5
        self.retry_counter = 0

        self.localization_transform = np.eye(4)

        self.safe_distance_lin=0.002
        self.safe_distance_ori=0.010

        self.acceptable_camera_delay_steps = 2

        self.pose_curr_2_goal_pub = rospy.Publisher('/pose_curr_to_goal', PoseStamped, queue_size=0)

        rospy.sleep(1)

    def init_record(self, trigger, template_name: str):
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('platonics_dataset')
        if not os.path.exists(self._package_path + '/trajectories/' + template_name):
            return False
        self.buttons.start_listening()
        init_pos = self.robot.curr_pos
        perturbation = 0 
        print("Move robot to start recording.")
        for _ in range(3):
            self.robot.vibrate(0.2)
            rospy.sleep(0.4)
        while perturbation < trigger:
            perturbation = math.sqrt((self.robot.curr_pos[0]-init_pos[0])**2 + (self.robot.curr_pos[1]-init_pos[1])**2 + (self.robot.curr_pos[2]-init_pos[2])**2)
                # trigger for starting the recording
        self.robot.set_stiffness(0, 0, 0, 0, 0, 0, 0)
        self.robot.set_K.update_configuration({"joint_default_damping": 1.00})
        self.recorded_pose = [self.robot.curr_pose]
        self.recorded_img = [self.camera.curr_image]
        self.recorded_img_feedback_flag = [False]
        self.recorded_spiral_flag = [False]
        self.recorded_compensation_flag = [False]
        if self.robot.gripper_width < self.grip_open_width * 0.9:
            self.buttons.gripper_closed = True
            self.gripper_state = True
        else:
            self.buttons.gripper_closed = False
            self.gripper_state = False
        self.recorded_gripper =  [self.grip_close_width if self.buttons.gripper_closed else self.grip_open_width]

        print("Recording started. Press Esc to stop.")

        self.buttons.end = False
        return True

    def kinesthetic_teaching(self, trigger=0.005):
        self.init_record(trigger=trigger)
        while not self.buttons.end:
            self.record_step()
        self.end_record()

    def end_record(self):
        self.robot.vibrate(0.2)
        self.buttons.stop_listening()
        goal = self.robot.curr_pose
        goal.header = Header(seq=1, stamp=rospy.Time.now(), frame_id="map")
        self.robot.goal_pub.publish(goal)
        #self.set_stiffness_execution()
        rospy.loginfo("Ending trajectory recording")
        self.data = {
            'recorded_pose': self.recorded_pose,
            'recorded_gripper': self.recorded_gripper,
            'recorded_img': self.recorded_img,
            'recorded_img_feedback_flag': self.recorded_img_feedback_flag,
            'recorded_spiral_flag': self.recorded_spiral_flag,
            'recorded_compensation_flag': self.recorded_compensation_flag
        }

    def record_step(self):
    
        if self.buttons.pressed:
            rospy.loginfo("Button pressed.")
            self.buttons.pressed=False
            self.robot.vibrate(0.2)
            
        if self.buttons.pause and not(self.buttons.end):
            self.rate.sleep()
            return
        poses=  interpolate_poses(self.recorded_pose[-1],self.robot.curr_pose, 0.003, self.safe_distance_ori)[1:]
        
        grip_value = self.grip_close_width if self.buttons.gripper_closed else self.grip_open_width
        change_gripper_state = self.gripper_state != self.buttons.gripper_closed
        self.gripper_state = self.buttons.gripper_closed
        # print("Change gripper state: ", change_gripper_state)
        if change_gripper_state:
            self.activate_gripper(grip_value)
        
        if self.buttons.change_in_stiff_rotation:
            if self.buttons.stiff_rotation:
                self.robot.goal_pub.publish(self.robot.curr_pose)
                self.robot.set_stiffness(0, 0, 0, 50, 50, 50, 0)
            else:
                self.robot.set_stiffness(0, 0, 0, 0, 0, 0, 0)
            self.buttons.change_in_stiff_rotation=False

        self.recorded_pose.extend(poses)
        self.recorded_gripper.extend([grip_value]*len(poses))
        self.recorded_img.extend([self.camera.curr_image]*len(poses))
        self.recorded_img_feedback_flag.extend([self.buttons.img_feedback_flag]*len(poses))
        self.recorded_spiral_flag.extend([self.buttons.spiral_flag]*len(poses))
        self.recorded_compensation_flag.extend([self.buttons.compensation_flag]*len(poses))
        self.rate.sleep()



    def execute(self, retry_insertion_flag=0):
        self.execute_start()
        while not(self.buttons.end):
            end_execute = self.execute_step(retry_insertion_flag)
            if end_execute:
                break
        self.execute_end()

    def set_stiffness_spiral(self):
        self.robot.set_stiffness(2000, 2000, 1000, 40, 40, 40, 0)

    def set_stiffness_execution(self):
        self.robot.set_stiffness(4000, 4000, 4000, 40, 40, 40, 0)

    def set_stiffness_safe(self):
        self.robot.set_stiffness(self.robot.K_pos_safe, self.robot.K_pos_safe, self.robot.K_pos_safe, self.robot.K_ori_safe, self.robot.K_ori_safe, self.robot.K_ori_safe, 0)

    def execute_start(self):
        self.buttons.start_listening()

        rospy.loginfo("Executing trajectory.")
        self.rate=rospy.Rate(self.control_rate*SPEED_FACTOR)

        self.total_transform = self.localization_transform
        self.compensation_transform = np.eye(4)
        self.servoing_transform = np.eye(4)
        self.spiral_transform = np.eye(4)
        self.robot.go_to_pose_ik(transform_pose(self.data['recorded_pose'][0],self.total_transform), interp_dist=0.003, interp_dist_joint=0.015) 
        self.set_stiffness_execution()
        self.robot.set_K.update_configuration({"max_delta_lin": 0.05})
        self.robot.set_K.update_configuration({"max_delta_ori": 0.50}) 
        self.robot.set_K.update_configuration({"joint_default_damping": 0.00})
        #self.robot.offset_compensator(10)

        self.time_index=0

        # Activate the gripper
        if self.robot.gripper_width < self.grip_open_width * 0.9:
            self.gripper_state = True
        else:
            self.gripper_state = False
        if self.data['recorded_gripper'][self.time_index] < self.grip_open_width * 0.9:
            self.buttons.gripper_closed = True
        else:
            self.buttons.gripper_closed = False
        change_gripper_state = self.gripper_state != self.buttons.gripper_closed
        self.gripper_state = self.buttons.gripper_closed
        if change_gripper_state:
            self.activate_gripper(self.data['recorded_gripper'][self.time_index])

        self.buttons.end = False


    def execute_step(self, retry_insertion_flag) -> bool:
        self.data= self.buttons.human_feedback(self.data, self.time_index)
        if self.time_index % np.floor(self.control_rate/self.space_nav_feedback._rate) == 0:
            self.data = self.space_nav_feedback.human_feedback(self.data, self.time_index)

        current_time=rospy.Time.now().to_sec()
        camera_delay = current_time-self.camera.time


        ### Perform compensation
        if self.time_index > self.window_size_steps:
            attractor_change_rate = np.linalg.norm(np.array(list(self.data['recorded_pose'][self.time_index].pose.position)) - np.array(list(self.data['recorded_pose'][max(0, self.time_index-self.window_size_steps)].pose.position)))
        else:
            attractor_change_rate = 1
        #if attractor_change_rate < 5e-3 and self.time_index % np.floor(self.control_rate/self.compensation_rate) == 0 and self.data['recorded_compensation_flag'][self.time_index]:
        if self.time_index % np.floor(self.control_rate/self.compensation_rate) == 0 and self.data['recorded_compensation_flag'][self.time_index]:
            curr_pose = deepcopy(self.robot.curr_pose)
            rospy.loginfo("Compensating")
            goal_pose = deepcopy(transform_pose(self.data['recorded_pose'][self.time_index], self.total_transform))
            transform_curr_pose_2_goal = transform_between_poses(goal_pose, curr_pose) 
            pose_curr_pose_2_goal = transformation_2_pose(transform_curr_pose_2_goal)
            self.pose_curr_2_goal_pub.publish(pose_curr_pose_2_goal)
            self.compensation_transform = transform_curr_pose_2_goal @ self.compensation_transform
            # self.compensation_transform[:3,3]=0
        # elif attractor_change_rate > 5e-4:
        #     self.compensation_transform = np.eye(4)
        elif self.data['recorded_compensation_flag'][self.time_index] == False:
            self.compensation_transform = np.eye(4)
        ### Perform camera corrections
        if self.data['recorded_img_feedback_flag'][self.time_index] and not self.camera.starting and (camera_delay * self.control_rate) < self.acceptable_camera_delay_steps and self.time_index % np.floor(self.control_rate/self.camera._rate) == 0:
            self.servoing_transform=self.camera.sift_matching(target_img=self.data['recorded_img'][self.time_index])
            self.total_transform = self.servoing_transform @ self.total_transform
        
        ### Perform spiral search
        if self.data['recorded_spiral_flag'][self.time_index] and not(self.buttons.pressed):
            if self.robot.force.z > self.insertion_force_threshold:
                self.spiral_transform = self.spiral_search(self.robot.goal_pose) #goal_pose)
                self.total_transform = self.spiral_transform @ self.total_transform

        ### Retry check
        force = np.linalg.norm([self.robot.force.x, self.robot.force.y, self.robot.force.z])
        if retry_insertion_flag and force > self.insertion_force_threshold:
            if self.retry_counter >= self.retry_limit:
                self.robot.move_gripper(self.grip_open_width)
                return True
            self.robot.go_to_pose(transform_pose(self.data['recorded_pose'][0],self.total_transform)) 
            self.time_index = 0
            self.retry_counter = self.retry_counter + 1
        ### Safety check
        if self.robot.safety_check: self.time_index += 1
        ### Publish the goal pose
        self.time_index = min(self.time_index, len(self.data['recorded_pose'])-1)
        goal_pose = self.data['recorded_pose'][self.time_index]
        goal_pose.header = {"seq": 1, "stamp": rospy.Time.now(), "frame_id": "map"}

        goal_pose=transform_pose(goal_pose, self.compensation_transform @ self.total_transform)
        self.robot.goal_pub.publish(goal_pose) 

        # Activate the gripper
        if self.robot.gripper_width < self.grip_open_width * 0.9:
            self.gripper_state = True
        else:
            self.gripper_state = False
        if self.data['recorded_gripper'][self.time_index] < self.grip_open_width * 0.9:
            self.gripper_state_recording = True
        else:
            self.gripper_state_recording = False

        change_gripper_state = self.gripper_state != self.gripper_state_recording
        if change_gripper_state:
            self.activate_gripper(self.data['recorded_gripper'][self.time_index])

        if self.time_index == (len(self.data['recorded_pose'])-1):
            return True

        self.rate.sleep()
        return False

    def execute_end(self):
        self.buttons.stop_listening()

    def abort(self):
        self.robot.stop_gripper()
        self.set_stiffness_safe()



    def activate_gripper(self, grip_value):
        if grip_value < self.grip_open_width * 0.9:
            self.robot.grasp_gripper(grip_value)
            time.sleep(0.5)
        else: 
            self.robot.move_gripper(grip_value)     
            time.sleep(0.5)

    def spiral_search(self, goal_pose: PoseStamped, force_min_exit=1): 
        # force_min_exit in Newton. If the verical force is lower than this value, the spiral search is considered successful
        max_spiral_time = 30 # seconds
        increase_radius_per_second = 0.0005 # meters, distance from center of the spiral after 1 second
        rounds_per_second = 1 # how many rounds does the spiral every second
        dt = 1. / self.control_rate

        goal_start = deepcopy(goal_pose)
        goal_final = deepcopy(goal_pose)
        time= 0 
        self.set_stiffness_spiral()
        for _ in range(max_spiral_time * self.control_rate):   
            if self.buttons.end:
                break
            goal_final.pose.position.x = goal_start.pose.position.x + np.cos(
                2 * np.pi *rounds_per_second*time) * increase_radius_per_second * time
            goal_final.pose.position.y = goal_start.pose.position.y + np.sin(
                2 * np.pi *rounds_per_second* time) * increase_radius_per_second * time
            self.robot.goal_pub.publish(goal_final)
            if self.robot.force.z <= force_min_exit: 
                break
            time += dt
            self.rate.sleep()
        self.set_stiffness_execution()
        trasform = transform_between_poses(goal_final, goal_start ) 

        return trasform

    def list_all_available_trajectories(self, template_name: str) -> List[str]:
        """List all available trajectories in the package

        All valid trajectories are stored in
        platonics_dataset/trajectories/<template> folder
        with the ending pkl. This function lists all the available trajectories

        """
        # Alll files in trajoctory data ending with pkl
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('platonics_dataset')
        files = os.listdir(self._package_path + '/trajectories/' + template_name)
        trajectories = [f.split('.')[0] for f in files if f.endswith('.pkl')]
        return trajectories

    def list_all_available_skills(self, template_name: str) -> List[str]:
        """List all available skills in the package

        All valid trajectories are stored in
        platonics_dataset/trajectories/<template> folder
        with the ending pkl. This function lists all the available trajectories

        """
        # Alll files in trajoctory data ending with pkl
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('platonics_dataset')
        files = os.listdir(self._package_path + '/trajectories/' + template_name)
        trajectories = [f.split('.')[0] for f in files if f.endswith('.yaml')]
        return trajectories

    def list_all_available_templates(self) -> List[str]:
        """List all available templates in the package

        All valid templates are stored in
        platonics_dataset/trajectories folder
        with the ending yaml. This function lists all the available templates

        """
        # Alll files in trajoctory data ending with pkl
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('platonics_dataset')
        template_folders = os.listdir(self._package_path + '/trajectories')
        # exclude files
        template_folders = [f for f in template_folders if not '.' in f]

        return template_folders



    def save(self, file : str, template_name : str):
        print("Saving trajectory")
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('platonics_dataset')
        with open(self._package_path + '/trajectories/' + template_name + '/' + file + '.pkl', 'wb') as f:
            pickle.dump(self.data, f)
        print("Trajectory saved")
        
    def load(self, file : str, template_name: str):
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('platonics_dataset')
        with open(self._package_path + '/trajectories/' + template_name + '/' + file + '.pkl', 'rb') as f:
            data = pickle.load(f)
        self.data = data

