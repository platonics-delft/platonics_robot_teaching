import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from feedback import Feedback
from utils import square_exp

class FeedbackSpacenav(Feedback):
    
    def __init__(self):
        super().__init__()
        self.feedback=np.zeros(3)
        self.feedback_gain=0.002
        self.length_scale = 0.005
        self.correction_window = 100
        self.feedback_treshold=0.001
        self.feedback_gain = 0.002
        # define a subscriber to space navigator
        self.subscriber = rospy.Subscriber('/spacenav/offset', Vector3, self.offset_subscriber)
    def offset_subscriber(self, data: Vector3):
        self.feedback[0] = data.x*self.feedback_gain
        self.feedback[1] = data.y*self.feedback_gain
        self.feedback[2] = data.z*self.feedback_gain

    def human_feedback(self, data: dict, time_index: int):
        if np.linalg.norm(self.feedback)>self.feedback_treshold:
            length = len(data['recorded_pose'])
            start = max(0, time_index-self.correction_window)
            end = min(length, time_index+self.correction_window)
            for j in range(start, end):
                se=square_exp(data['recorded_pose'][time_index],data['recorded_pose'][j], self.length_scale)
                delta_x = self.feedback[0]*se
                delta_y = self.feedback[1]*se
                delta_z = self.feedback[2]*se

                data['recorded_pose'][j].pose.position.x += delta_x
                data['recorded_pose'][j].pose.position.y += delta_y
                data['recorded_pose'][j].pose.position.z += delta_z
        return data
