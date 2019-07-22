# Move_and_grasp
The whole ROS workspace for move and adjust robot to grasp objects

---
## Prerequirements

moveIt!

turtlebot_arm

astra

for environment setup see

http://robotforall.org/wiki/index.php?title=Vision_System_Setup

http://robotforall.org/wiki/index.php?title=TurtleBot_Arm_setup

---
## Usage

bringup the TurtleBot, astra and arm:

`
roslaunch move_and_grasp start_turtle.launch
`

run the object_detection and odom_adjust and arm node:

`
roslaunch move_and_grasp start_grasp_attemp.launch
`

click center of the object you want to grasp in the "find_object_window" 

wait for the robot to grasp









