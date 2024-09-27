## Platonics Equipment
- Franka Emika Panda robot aka *Aristotle*
- The Franka Hand with two-finger gripper
- Gripper inlet. [Download STL](./assets/finger_tips.STL)
- [Intel NUC kit NUC10i7FNHN2](https://www.coolblue.nl/product/858939/intel-nuc-kit-nuc10i7fnhn2.html)
- [Intel Realsense Camera D405](https://www.intelrealsense.com/depth-camera-d405/)
- Camera mount. [Download STL](./assets/Camera_mount_realsense.STL)

## Software Overview

Our solution to is made fully open-source to allow the community to build on top and further improve our solution. Most components are stored at [Platonics Delft](https://github.com/orgs/platonics-delft). You can find all required software components and links to their installation guides below. The first repo is the controller. Follow its instructions to install it on the host computer. The remaining packages must be cloned into the catkin workspace on the computer that is connected to the camera and to the robot by an Ethernet connection.

1. [Franka Human Friendly Controllers](https://github.com/franzesegiovanni/franka_human_friendly_controllers)
2. [Platonics Robot Teaching](https://github.com/platonics-delft/platonics_robot_teaching)
3. [Panda Utils](https://github.com/platonics-delft/panda-ros-py)
4. [Platonics Vision](https://github.com/platonics-delft/platonics_vision)
5. [Platonics Dataset](https://github.com/platonics-delft/platonics_dataset)
6. [Platonics GUI](https://github.com/platonics-delft/platonics_gui)
7. [Platonics Tools](https://github.com/platonics-delft/platonics_tools)

## Installation

Follow the instructions here to install the controller on the computer connected to the robot:
[Franka Human Friendly Controllers](https://github.com/franzesegiovanni/franka_human_friendly_controllers)

Follow the instructions to install the vision packages on
[Platonics Vision](https://github.com/platonics-delft/platonics_vision)

You can run the following commands to install the [Panda Utils](https://github.com/platonics-delft/panda-ros-py) package (which should also be in the same directory as the rest of the packages 
on the computer connected to the camera and to the robot via ethernet):
```
cd robot_ws
mkdir src
cd src
git clone https://github.com/platonics-delft/panda-ros-py.git
cd src
pip install -r requirements.txt
cd ../..
catkin build
source devel/setup.bash
```

Follow the instructions to install the GUI packages on
[Platonics GUI](https://github.com/platonics-delft/platonics_gui).

The rest of the repos simply need to be cloned (platonics_tools and platonics_dataset) in the same workspace.

And finally, the instructions on [Platonics GUI](https://github.com/platonics-delft/platonics_gui)
indicate how to launch everything with a few simple commands. 

Please remember to source the workspace in every terminal you open.

## (Optinally, without the GUI) Run only the services included in this repository

Ensure that the franka human friendly variable cartesian impedance controller is
running.

Run the learning from demonstration services
```bash
rosrun skills_manager lfd_servers
```
Then, 4 services become available:
- \lfdHome: Homing the robot
- \lfdRecord: Record a skill given a template
- \lfdExecuteSequence: Execute a sequence of recordings
- \lfdExecuteSkill: Execute a skill which can contain multiple sequences or
  other skills

## Recording

During recording you can give certain flags to the robot by using the buttons
on the robot. The flags are:
- 'Up': Pause
- 'Down': Spiraling
- 'Right': Offset compenestation
- 'Checkmark': Camera Feedback
- 'Cross': Stop recording
- 'Circle': Gripper Toggle

