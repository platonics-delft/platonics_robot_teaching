import rospy
import numpy as np
class Feedback():
    def __init__(self):
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
        self.stiff_rotation = False
        self.gripper_closed = False
        self.feedback = np.zeros(4)


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
        (recorded_pose[ind_curr].pose.position.x - recorded_pose[ind_j].pose.position.x) ** 2 +
        (recorded_pose[ind_curr].pose.position.y - recorded_pose[ind_j].pose.position.y) ** 2 +
        (recorded_pose[ind_curr].pose.position.z - recorded_pose[ind_j].pose.position.z) ** 2
    )
    sq_exp = np.exp(-dist ** 2 / self.length_scale ** 2)
    return sq_exp 
    