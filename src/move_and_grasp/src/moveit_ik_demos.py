#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
    def __init__(self):
        rospy.init_node('grasp_attempt')
        rospy.Subscriber("/adjust_to_arm", String, self.navCallback)
        self.arm_pub = rospy.Publisher("/arm_to_control", String, queue_size=1)
        rospy.spin()
        
    def navCallback(self, msg):
        if msg.data == 'start_grasp':
            self.grasp_attempt()


    def grasp_attempt(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        GRIPPER_OPEN = [-0.3]
        GRIPPER_CLOSED = [0.4]
        GRIPPER_NEUTRAL = [0.4]
                
        # Initialize the move group for the right arm
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # Connect to the right_gripper move group
        gripper = moveit_commander.MoveGroupCommander('gripper')
       
        # Allow 5 seconds per planning attempt
        arm.set_planning_time(5)

        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'base_link'
        
        # Set the right arm reference frame accordingly
        arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.03)
        arm.set_goal_orientation_tolerance(0.04)
        
        # Set the gripper target to neutal position using a joint value target
        gripper.set_joint_value_target(GRIPPER_OPEN)
         
        # Plan and execute the gripper motion
        gripper.go()
        rospy.sleep(2)
        rospy.loginfo("gripper open ")

        # Set the arm target to the named "right_up" pose stored in the SRDF file
        #arm.set_named_target('right_up')
         
        # Plan the trajectory to the goal
        #traj = arm.plan()
        
        # Execute the planned trajectory
        #arm.execute(traj)
        
        # Pause for a moment
        #rospy.sleep(3)
        #rospy.loginfo("ready")
               
        # Set the target pose.  This particular pose has the gripper oriented horizontally
        # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
        # the center of the robot base.
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.3456
        target_pose.pose.position.y = 0.01619
        target_pose.pose.position.z = 0.91166
        
        # Set the start state to the current state
        arm.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        arm.set_pose_target(target_pose, end_effector_link)
        
        # Plan the trajectory to the goal
        traj = arm.plan()
        
        # Execute the planned trajectory
        arm.execute(traj)
    
        # Pause for a second
        rospy.sleep(7)
         
        # Close the gripper as if picking something up
        gripper.set_joint_value_target(GRIPPER_CLOSED)
        gripper.go()
        rospy.sleep(1)
        rospy.loginfo("catch it,right?")
                  
        # Set the arm target to the named "right_up" pose stored in the SRDF file
        arm.set_named_target('right_up')
         
        # Plan the trajectory to the goal
        traj = arm.plan()
        
        # Execute the planned trajectory
        arm.execute(traj)
        rospy.loginfo("up!!!up!!!")
        rospy.sleep(7)
       
        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        joint_positions = [0, 0, 1.3, 0, 0]
 
        # Set the arm's goal configuration to the be the joint positions
        arm.set_joint_value_target(joint_positions)
                 
        # Plan and execute the motion
        arm.go()
        rospy.sleep(2)   
        rospy.loginfo(" open gripper")   

        # Open the gripper as if letting something go
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(3)   
        rospy.loginfo("enopen now!")    

        # Return the arm to the named "resting" pose stored in the SRDF file
        arm.set_named_target('right_up')
        #Plan the trajectory to the goal
        traj = arm.plan()
        
        # Execute the planned trajectory
        arm.execute(traj)
        rospy.sleep(7)
         
        # Return the gripper target to neutral position
        gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        gripper.go()
        rospy.sleep(2)
        rospy.loginfo("tadayima")

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    
