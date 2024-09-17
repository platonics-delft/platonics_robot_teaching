import numpy as np
from utils import square_exp
class Feedback():
    def __init__(self):
        self.feedback_treshold = 0.001
        self.length_scale = 0.005
        self.correction_window = 200
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
        self.feedback=np.zeros(3)
        self.change_in_stiff_rotation = False


    def start_listening(self):
        pass

    def stop_listening(self):
        pass


    def human_feedback(self, data: dict, time_index: int):
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
            print("hi")
            print(self.img_feedback_flag)
            data['recorded_img_feedback_flag'][time_index:] = [self.img_feedback_flag] * len(data['recorded_img_feedback_flag'][time_index:])


        if self.spiral_feedback_correction:
            data['recorded_spiral_flag'][time_index:] = [self.spiral_flag] * len(data['recorded_pose'][time_index:])

        if np.linalg.norm(self.feedback) > self.feedback_treshold:
            length = len(data['recorded_pose'])
            start = max(0, time_index-self.correction_window)
            end = min(length, time_index+self.correction_window)
            for j in range(start, end):
                square_exp=square_exp(data['recorded_pose'][time_index],data['recorded_pose'][j], self.length_scale)
                delta_x = self.feedback[0]* square_exp
                delta_y = self.feedback[1]* square_exp
                delta_z = self.feedback[2]* square_exp

                data['recorded_pose'][j].pose.position.x += delta_x
                data['recorded_pose'][j].pose.position.y += delta_y
                data['recorded_pose'][j].pose.position.z += delta_z
            return data
        
        self.feedback=np.zeros(3)
        self.speed_up = False                
        self.img_feedback_correction = False
        self.spiral_feedback_correction = False

        return data 
    
