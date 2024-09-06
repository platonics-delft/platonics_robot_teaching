from feedback import Feedback
import typing
import panda_py
import rospkg
import yaml
class FeedbackButtons(Feedback):
    def __init__(self):
        super().__init__()

        # Connect to the Desk
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('skills_manager')
        with open(self._package_path + '/config/login.yaml', 'rb') as f:
            login_info = yaml.safe_load(f)
        hostname = login_info['robot_ip']
        username = login_info['username']
        password = login_info['password']
        self.desk = panda_py.Desk(hostname, username, password)

    def start_listening(self):
        self.reset_variables()
        self.desk.listen(self.on_press)
        self.desk._listening = True

    def stop_listening(self):
        self.desk._listening = False

    def on_press(self, event: typing.Dict) -> None:
        read_events=list(event.keys())
        for i in range(len(read_events)):
            if read_events[i] == 'up':
                if event[read_events[i]] == True and not self.blocked:
                    self.blocked = True
                    self.pressed=True
                    self.pause=not(self.pause)
                    if self.pause==True:
                        print("Recording paused")    
                    else:
                        print("Recording started again")
                elif event[read_events[i]] == False:    
                    self.blocked = False        
            if read_events[i] == 'down':
                if event[read_events[i]] == True and not self.blocked:
                    self.blocked = True
                    self.pressed=True
                    if self.spiral_flag:
                        print("spiral disabled")
                        self.spiral_flag = False
                        self.spiral_feedback_correction = True
                    else:
                        print("spiral enabled")
                        self.spiral_flag = True
                        self.spiral_feedback_correction = True
                elif event[read_events[i]] == False:
                    self.blocked = False

            if read_events[i] == 'left':
                if event[read_events[i]] == True and not self.blocked:
                    self.blocked = True
                    self.pressed=True
                    if self.stiff_rotation:
                        print("Stiff rotation disabled")
                        self.stiff_rotation = False
                        self.change_in_stiff_rotation=True
                    else:
                        print("Stiff rotation enabled")
                        self.stiff_rotation = True
                        self.change_in_stiff_rotation=True
                elif event[read_events[i]] == False:
                    self.blocked = False

            if read_events[i] == 'right':
                if event[read_events[i]] == True and not self.blocked:
                    self.blocked = True
                    self.pressed=True
                    print("Speed up")
                    self.speed_up = True
                elif event[read_events[i]] == False:
                    self.blocked = False

            if read_events[i] == 'circle':
                if event[read_events[i]] == True and not self.blocked:
                    self.blocked = True
                    self.pressed=True
                    if self.gripper_closed:  
                        print("Gripper open")
                        self.gripper_closed = False
                    else:
                        self.gripper_closed =True
                        print("Gripper closed")
                elif event[read_events[i]] == False:
                    self.blocked = False
            if read_events[i] == 'cross':
                if event[read_events[i]] == True and not self.blocked:
                    self.blocked = True
                    self.pressed=True
                    self.end = True
                    print("Esc pressed. Stopping...")
                    # Stop listening
                    self.desk._listening = False
                elif event[read_events[i]] == False:
                    self.blocked = False
            if read_events[i] == 'check':
                if event[read_events[i]] == True and not self.blocked:
                    self.blocked = True
                    self.pressed=True
                    if self.img_feedback_flag:
                        print("camera feedback disabled")
                        self.img_feedback_flag = False
                        self.img_feedback_correction = True
                    else:
                        print("camera feedback enabled")
                        self.img_feedback_flag = True
                        self.img_feedback_correction = True
                elif event[read_events[i]] == False:
                    self.blocked = False
                    
