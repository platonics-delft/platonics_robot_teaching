import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from feedback import Feedback
from utils import square_exp

class FeedbackSpacenav(Feedback):
    
    def __init__(self):
        super().__init__()
        self.feedback=np.zeros(3)
        self.feedback_gain=0.0005
        self.length_scale = 0.005
        self.correction_window = 100
        self.feedback_gain = 0.0005
        self.feedback_treshold=self.feedback_gain/10
        self._rate = 10
        # define a subscriber to space navigator
        self.subscriber = rospy.Subscriber('/spacenav/offset', Vector3, self.offset_subscriber)
    def offset_subscriber(self, data: Vector3):
        self.feedback[0] = data.x*self.feedback_gain
        self.feedback[1] = data.y*self.feedback_gain
        self.feedback[2] = data.z*self.feedback_gain

    def human_feedback(self, data: dict, time_index: int):
        if np.linalg.norm(self.feedback)>self.feedback_treshold:
            length = len(data['recorded_pose'])
            increment = self.feedback
            print(increment)
            for j in range(time_index, length):
                data['recorded_pose'][j].pose.position.x += increment[0]
                data['recorded_pose'][j].pose.position.y += increment[1]
                data['recorded_pose'][j].pose.position.z += increment[2]
        return data
