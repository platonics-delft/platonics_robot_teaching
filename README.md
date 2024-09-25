# Platonics Robot Teaching
Our solution to Robot Teaching is made fully open-source to allow the community
to build on top and further improve our solution. All components are stored in
[Platonics Delft](https://github.com/orgs/platonics-delft). 

## Installation

Follow the instructions here to install the controller on the computer connected to the robot:
[Franka Human Friendly Controllers](https://github.com/franzesegiovanni/franka_human_friendly_controllers)

Follow the instructions to install the vision packages on
[Platonics Vision](https://github.com/platonics-delft/platonics_vision)

```
mkdir robot_ws
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

Please remember to source the workspace in every terminal you open.

## Getting Started

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

