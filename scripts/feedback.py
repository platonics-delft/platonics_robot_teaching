import rospy
import numpy as np
from pynput.keyboard import KeyCode, Key
from pynput.keyboard import Listener
from panda_ros.pose_transform_functions import array_quat_2_pose, list_2_quaternion
class Feedback():
    def __init__(self):
        super(Feedback, self).__init__()
        self.feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.faster_counter=0
        self.length_scale = 0.005
        self.correction_window = 300
        self.img_feedback_flag = 0
        self.spiral_flag = 0
        self.img_feedback_correction = 0
        self.gripper_feedback_correction = 0
        self.spiral_feedback_correction=0
        self.pause=False
        self.rigid =0
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        rospy.loginfo(f"Event happened, user pressed {key}")
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.esc:
            self.end = True
            print("Esc pressed. Stopping...")
        if key == KeyCode.from_char('w'):
            self.feedback[0] = self.feedback_gain
        if key == KeyCode.from_char('s'):
            self.feedback[0] = -self.feedback_gain
        # Feedback for translate left/right
        if key == KeyCode.from_char('a'):
            self.feedback[1] = self.feedback_gain
        if key == KeyCode.from_char('d'):
            self.feedback[1] = -self.feedback_gain
        # Feedback for translate up/down
        if key == KeyCode.from_char('u'):
            self.feedback[2] = self.feedback_gain
        if key == KeyCode.from_char('j'):
            self.feedback[2] = -self.feedback_gain
        # Close/open gripper
        if key == KeyCode.from_char('c'):
            if self.gripper_opened:   
                self.panda.grasp_gripper(self.grip_close_width)
                print("Gripper closed")
                # self.gripper_feedback_correction = 1
            else:
                self.panda.move_gripper(self.grip_open_width)
                print("Gripper opened")
                # self.gripper_feedback_correction = 1
        if key == KeyCode.from_char('f'):
            self.feedback[3] = 1
        if key == KeyCode.from_char('k'):
            if self.img_feedback_flag == 0:
                print("camera feedback enabled")
                self.img_feedback_flag = 1
                self.img_feedback_correction = 1
            else:
                print("camera feedback disabled")
                self.img_feedback_flag = 0
                self.img_feedback_correction = 1
        if key == KeyCode.from_char('z'):
            if self.spiral_flag == 0:
                print("spiral enabled")
                self.spiral_flag = 1
                self.spiral_feedback_correction = 1
            else:
                print("spiral disabled")
                self.spiral_flag = 0
                self.spiral_feedback_correction = 1

        if key == KeyCode.from_char('m'):    
            if self.rigid == 0:
                self.goal_pub.publish(self.curr_pose)
                self.set_stiffness(0, 0, 0, 50, 50, 50, 0)
                self.stiffness = 1
                print("higher rotational stiffness")
                self.rigid = 1
            else:
                self.set_stiffness(0, 0, 0, 0, 0, 0, 0)
                print("zero rotational stiffness")
        if key == Key.space:
            self.pause=not(self.pause)
            if self.pause==True:
                print("Recording paused")    
            else:
                print("Recording started again")  
        key=None
    def square_exp(self, ind_curr, ind_j):
        dist = np.sqrt(
            (self.recorded_pose[ind_curr].pose.position.x - self.recorded_pose[ind_j].pose.position.x) ** 2 +
            (self.recorded_pose[ind_curr].pose.position.y - self.recorded_pose[ind_j].pose.position.y) ** 2 +
            (self.recorded_pose[ind_curr].pose.position.z - self.recorded_pose[ind_j].pose.position.z) ** 2
        )
        sq_exp = np.exp(-dist ** 2 / self.length_scale ** 2)
        return sq_exp 

    def human_feedback(self):
        if np.sum(self.feedback[:3])!=0:
            for j in range(self.recorded_traj.shape[1]):
                x = self.feedback[0]*self.square_exp(self.time_index, j)
                y = self.feedback[1]*self.square_exp(self.time_index, j)
                z = self.feedback[2]*self.square_exp(self.time_index, j)

                self.recorded_pose[j].pose.position.x += x
                self.recorded_pose[j].pose.position.y += y
                self.recorded_pose[j].pose.position.z += z
        
        if self.img_feedback_correction:
            self.recorded_img_feedback_flag[self.time_index:] = self.img_feedback_flag

        if self.spiral_feedback_correction:
            self.recorded_spiral_flag[self.time_index:] = self.spiral_flag
        
        if self.feedback[3] != 0:
            self.faster_counter = 10
            
        if self.faster_counter > 0 and self.time_index != self.recorded_traj.shape[1]-1:
            self.faster_counter -= 1

            self.recorded_pose.pop(self.time_index+1)
            self.recorded_gripper.pop(self.time_index+1)
            self.recorded_img.pop(self.time_index+1)
            self.recorded_img_feedback_flag.pop(self.time_index+1)
            self.recorded_spiral_flag.pop(self.time_index+1)
                       
        self.feedback = np.zeros(4)
        self.img_feedback_correction = 0
        self.spiral_feedback_correction = 0 

    