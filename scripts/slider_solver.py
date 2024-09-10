import rospy
from typing import List
import os
import math
import numpy as np
import time
import rospkg
from geometry_msgs.msg import PoseStamped
from panda_ros import Panda
from feedback_spacenav import FeedbackSpacenav
from panda_ros.pose_transform_functions import transform_pose, pos_quat_2_pose_st, pose_st_2_transformation
from copy import deepcopy
import pickle
import cv2
from cv_bridge import CvBridgeError, CvBridge
import datetime
from platonics_vision.scripts.triangle_detector import TriangleDetector

class SliderPusher():
    def __init__(self):
        self.control_rate = 30
        self.rate=rospy.Rate(self.control_rate)
        self.move_increment = 0.0001

        self.robot = Panda()
        self.triangle_detector = TriangleDetector()

        self.safe_distance_lin=0.005
        self.safe_distance_ori=0.020

        self.success_threshold = 5

        self.acceptable_camera_delay_steps = 2

        self.bridge = CvBridge()

        rospy.sleep(1)

    def image_callback(self, msg):
            # Convert the ROS message to a OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.curr_image = cv_image
        except CvBridgeError as e:
            print(e)
        
    def solve_slider(self, task_stage):
        self.execute_start(task_stage)
        while not(self.end):
            end_execute = self.execute_step(task_stage)
            if end_execute:
                break

    def execute_start(self, task_stage):
        rospy.loginfo("Executing trajectory.")
        self.rate=rospy.Rate(self.control_rate)

        self.robot.set_stiffness(3000, 3000, 3000, 30, 30, 30, 0)
        self.robot.set_K.update_configuration({"max_delta_lin": 0.05})
        self.robot.set_K.update_configuration({"max_delta_ori": 0.50}) 
        self.robot.set_K.update_configuration({"joint_default_damping": 0.00})

        self.robot.change_in_safety_check = False
        if task_stage == 1:
            object_ids = ['red', 'white_center']
        if task_stage == 2:
            object_ids = ['red', 'green']
        self.triangle_detector.load_template_images(image_dir_path='', object_ids=['red', 'white_center'])

    def execute_step(self, task_stage) -> bool:
        ### Safety check
        if self.robot.change_in_safety_check:
            if self.robot.safety_check:
                    self.robot.set_stiffness(self.robot.K_pos, self.robot.K_pos, self.robot.K_pos, self.robot.K_ori, self.robot.K_ori, self.robot.K_ori, 0)
            else:
                # print("Safety violation detected. Making the robot compliant")
                self.robot.set_stiffness(self.robot.K_pos_safe, self.robot.K_pos_safe, self.robot.K_pos_safe, self.robot.K_ori_safe, self.robot.K_ori_safe, self.robot.K_ori_safe, 0)
                return True
            
        self.triangles_distance = self.detect_triangles(task_stage)
        direction = np.sign(self.triangles_distance)
        curr_goal = self.robot.curr_goal
        new_goal_in_ee_frame = pos_quat_2_pose_st(np.array([0., direction*self.move_increment, 0.]), np.quaternion(1.0,0.,0.,0.))
        transform_world_ee = pose_st_2_transformation(curr_goal)
        new_goal_in_world_frame = transform_pose(new_goal_in_ee_frame, transform_world_ee)

        ### Publish the goal pose
        new_goal_in_world_frame.header = {"seq": 1, "stamp": rospy.Time.now(), "frame_id": "map"}
        self.robot.goal_pub.publish(new_goal_in_world_frame) 

        if np.abs(self.triangles_distance) < self.success_threshold:
            return True

        self.rate.sleep()
        return False