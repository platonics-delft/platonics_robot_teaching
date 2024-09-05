import rospy
import numpy as np
class Feedback():
    def __init__(self):
        self.feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.length_scale = 0.005
        self.correction_window = 300
        self.reset_variables()


    def reset_variables(self):
        self.pause=False
        self.img_feedback_correction = False
        self.gripper_feedback_correction = False
        self.spiral_flag = False
        self.spiral_feedback_correction = False
        self.stiff_rotation = False
        self.img_feedback_flag = False
        self.gripper_closed = False
        self.blocked = False
        self.pressed = False
        self.speed_up = False
        self.faster_counter=0


    def start_listening(self):
        pass

    def stop_listening(self):
        pass


    def human_feedback(self, data: dict, time_index: int):
        if np.sum(self.feedback[:3])!=0:
            for j in range(len(data['recorded_pose'])):
                x = self.feedback[0]*square_exp(data['recorded_pose'], time_index, j, self.length_scale)
                y = self.feedback[1]*square_exp(data['recorded_pose'], time_index, j, self.length_scale)
                z = self.feedback[2]*square_exp(data['recorded_pose'], time_index, j, self.length_scale)

                data['recorded_pose'][j].pose.position.x += x
                data['recorded_pose'][j].pose.position.y += y
                data['recorded_pose'][j].pose.position.z += z
                
        if self.speed_up:
            self.faster_counter = 10
            
        if self.faster_counter > 0 and time_index != len(data['recorded_pose'])-1:
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

def square_exp(recorded_pose, ind_curr, ind_j, length_scale):
    dist = np.sqrt(
        (recorded_pose[ind_curr].pose.position.x - recorded_pose[ind_j].pose.position.x) ** 2 +
        (recorded_pose[ind_curr].pose.position.y - recorded_pose[ind_j].pose.position.y) ** 2 +
        (recorded_pose[ind_curr].pose.position.z - recorded_pose[ind_j].pose.position.z) ** 2
    )
    sq_exp = np.exp(-dist ** 2 / length_scale ** 2)
    return sq_exp 
    
