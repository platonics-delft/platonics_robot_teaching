import rospy
import numpy as np
from pynput.keyboard import KeyCode, Key
from pynput.keyboard import Listener
from feedback import Feedback
class FeedbackKeyboard(Feedback):
    def __init__(self):
        super(FeedbackKeyboard, self).__init__()
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
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
        if key == KeyCode.from_char('c'):
            if self.img_feedback_flag:
                print("camera feedback disabled")
                self.img_feedback_flag = False
                self.img_feedback_correction = True
            else:
                print("camera feedback enabled")
                self.img_feedback_flag = True
                self.img_feedback_correction = True
        if key == KeyCode.from_char('x'):
            if self.spiral_flag:
                print("spiral disabled")
                self.spiral_flag = False
                self.spiral_feedback_correction = True
            else:
                print("spiral enabled")
                self.spiral_flag = True
                self.spiral_feedback_correction = True

        # Close/open gripper
        if key == KeyCode.from_char('g'):
            if self.gripper_closed:  
                print("Gripper open")
                self.gripper_closed = False
            else:
                self.gripper_closed =True
                print("Gripper closed")
        if key == KeyCode.from_char('z'):    
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