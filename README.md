# Move_and_grasp
The whole ROS workspace for move and adjust robot to grasp objects


## Prerequirements

moveIt!

turtlebot_arm

astra

for environment setup see

http://robotforall.org/wiki/index.php?title=Vision_System_Setup

http://robotforall.org/wiki/index.php?title=TurtleBot_Arm_setup

## Modify a pose for grasping

bring up the arm:

`
roslaunch turtlebot_arm_bringup myarm.launch
`

move the joints with keyboard:

`
rosrun turtlebot_arm_moveit_demos arm_keyboard_control.py
`

get arm posees:

`
rosrun turtlebot_arm_moveit_demos get_arm_posees.py
`

an example output:

```

['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint']

[INFO] [1563869335.417028]: Joint values:
[-0.06135923151542565, 0.35792885050664963, 0.6033657765683522, 0.5471198143458788, 0.0]

[INFO] [1563869335.468412]: End effector pose:
header: 
  seq: 0
  stamp: 
    secs: 1563869335
    nsecs: 465894937
  frame_id: "/base_footprint"
pose: 
  position: 
    x: 0.348105493
    y: 0.000539585021142
    z: 0.842247851812
  orientation: 
    x: -0.000946584552211
    y: -0.0307807765032
    z: -0.0306601945781
    w: 0.999055353944

```

Add the joint values here in moveit_fk_demos.py:

```python
# Set target joint values for the arm: joints are in the order they appear in
# the kinematic tree.
    joint_positions = [-0.06136, 0.358, 0.6034, 0.5471, 0.0]
```

moveit! will help plan a smooth way to arrive the expected joint positions

Also set the expected end_effector position x and y here in odom_adjust_for_grasp.py:

```python
# for fk  demo
    self.required_pos.x = 0.348105493
    self.required_pos.y  = 0.00053959
```





## Usage


bringup the TurtleBot, astra and arm:

`
roslaunch move_and_grasp start_turtle.launch
`

run the object_detection and odom_adjust and arm node:

`
roslaunch move_and_grasp start_grasp_attemp.launch
`

**click center of the object you want to grasp in the "find_object_window"** 

wait for the robot to grasp









