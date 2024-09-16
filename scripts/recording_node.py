#!/usr/bin/env python
import rospy
from datetime import datetime
from std_msgs.msg import Bool
from panda_ros import Panda
from camera_feedback import Camera
import rospkg
import pickle
class Recorder:
    def __init__(self):
        self.recording_freq = 20
        self.recording = False

        rospy.init_node("recording_node")
        rospy.Subscriber("/recording", Bool, self.recording_callback)

        self.camera = Camera()
        self.robot= Panda()

    def recording_callback(self, data):
        self.recording=data


    def record(self):
        print("Press m to start recording, press e to stop")
        r = rospy.Rate(self.recording_freq)
        timestamp = datetime.now().strftime("%d_%H_%M_%S")

        self.recorded_pose = [self.robot.curr_pose]
        self.recorded_goal = [self.robot.goal_pose]
        self.recorded_img = [self.camera.curr_image]
        self.recorded_wrench = [self.robot.curr_wrench]
        self.recorded_joint = [self.robot.curr_joint]


        while not self.recording:
            pass

        print('Recording')
        while self.recording:
            # print("Test")

            self.recorded_pose.append(self.robot.curr_pose)
            self.recorded_goal.append(self.robot.goal_pose)
            self.recorded_img.append(self.camera.curr_image)
            self.recorded_joint.append(self.robot.curr_joint) 
            self.recorded_wrench.append(self.robot.curr_wrench)

            r.sleep()
        self.data  = {
            'pose': self.recorded_pose,
            'goal': self.recorded_goal,
            'img': self.recorded_img,
            'joint': self.recorded_joint, 
            'wrench': self.recorded_wrench
        }
        self.save(filename=str(timestamp))


    def save(self, file='last'):
        print("Saving trajectory")
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_data')
        with open(self._package_path + '/recorded_trials/' + str(file) + '.pkl', 'wb') as f:
            pickle.dump(self.data, f)
        print("Data rollout saved")