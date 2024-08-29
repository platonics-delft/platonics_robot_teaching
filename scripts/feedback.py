import rospy
import numpy as np
from pynput.keyboard import KeyCode, Key
from pynput.keyboard import Listener
class Feedback():
    def __init__(self):
        super(Feedback, self).__init__()
        feedback=np.zeros(4)
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
        self.stiff_rotation = False
        self.gripper_closed = False
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
        if key == KeyCode.from_char('f'):
            self.feedback[3] = 1
        if key == KeyCode.from_char('k'):
            if self.img_feedback_flag:
                print("camera feedback disabled")
                self.img_feedback_flag = False
                self.img_feedback_correction = True
            else:
                print("camera feedback enabled")
                self.img_feedback_flag = True
                self.img_feedback_correction = True
        if key == KeyCode.from_char('z'):
            if self.spiral_flag:
                print("spiral disabled")
                self.spiral_flag = False
                self.spiral_feedback_correction = True
            else:
                print("spiral enabled")
                self.spiral_flag = True
                self.spiral_feedback_correction = True

        # Close/open gripper
        if key == KeyCode.from_char('c'):
            if self.gripper_closed:  
                print("Gripper open")
                self.gripper_closed = False
            else:
                self.gripper_closed =True
                print("Gripper closed")
        if key == KeyCode.from_char('m'):    
            if self.stiff_rotation:
                self.stiff_rotation = False
                print("zero rotational stiffness")
            else:
                self.stiff_rotation = True
                print("higher rotational stiffness")
        if key == Key.space:
            self.pause=not(self.pause)
            if self.pause==True:
                print("Recording paused")    
            else:
                print("Recording started again")  
        key=None

    def human_feedback(self, data: dict, time_index: int):
        if np.sum(self.feedback[:3])!=0:
            for j in range(len(data['recorded_pose'])):
                x = self.feedback[0]*square_exp(data['recorded_pose'], time_index, j)
                y = self.feedback[1]*square_exp(data['recorded_pose'], time_index, j)
                z = self.feedback[2]*square_exp(data['recorded_pose'], time_index, j)

                data['recorded_pose'][j].pose.position.x += x
                data['recorded_pose'][j].pose.position.y += y
                data['recorded_pose'][j].pose.position.z += z
                
        if self.feedback[3] != 0:
            self.faster_counter = 10
            
        if self.faster_counter > 0 and time_index != self.recorded_traj.shape[1]-1:
            self.faster_counter -= 1

            data['recorded_pose'].pop(time_index+1)
            data['recorded_gripper'].pop(time_index+1)
            data['recorded_img'].pop(time_index+1)
            data['recorded_img_feedback_flag'].pop(time_index+1)
            data['recorded_spiral_flag'].pop(time_index+1)

        if self.img_feedback_correction:
            data['recorded_img_feedback_flag'][time_index:] = self.img_feedback_flag

        if self.spiral_feedback_correction:
            data['recorded_spiral_flag'][time_index:] = self.spiral_flag            
        feedback = np.zeros(4)
        self.img_feedback_correction = False
        self.spiral_feedback_correction = False

        return data

def square_exp(recorded_pose, ind_curr, ind_j):
    dist = np.sqrt(
        recorded_pose[ind_curr].pose.position.x - recorded_pose[ind_j].pose.position.x) ** 2 +
        recorded_pose[ind_curr].pose.position.y - recorded_pose[ind_j].pose.position.y) ** 2 +
        recorded_pose[ind_curr].pose.position.z - recorded_pose[ind_j].pose.position.z) ** 2
    )
    sq_exp = np.exp(-dist ** 2 / self.length_scale ** 2)
    return sq_exp 
    