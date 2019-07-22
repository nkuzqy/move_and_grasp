#!/usr/bin/env python

# new calculating algorithm
import rospy
import math
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
from std_msgs.msg import String
class Loop:
	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		init_pose = (1.4,1.57,-1.57,0.6,-0.5)
		#init_pose = (0,1.57,0.7,0.7,0.5)
		hold_pose = (0,-2,2,-0.3,0)

		set_pose = (0,1.57,0.7,0.7,0.5)

		self.joint1 = rospy.Publisher('/arm_shoulder_pan_joint/command',Float64)
		self.joint2 = rospy.Publisher('/arm_shoulder_lift_joint/command',Float64)
		self.joint3 = rospy.Publisher('/arm_elbow_flex_joint/command',Float64)
		self.joint4 = rospy.Publisher('/arm_wrist_flex_joint/command',Float64)
		self.joint5 = rospy.Publisher('/gripper_joint/command',Float64)
		
		self.released_pub = rospy.Publisher('/arm_released', String)

		# the variable for diandong tuigan
		self.mark = 0
		def callback_carry (data_nav):
			if data_nav.data.find('grasp') > -1:
				rospy.loginfo( "I heard that the robot has reached point B")
				self.mark = 1
			if data_nav.data.find('release') > -1:
				self.mark = 2

		# adding program above
		rospy.Subscriber("nav2arm",String, callback_carry)
		rospy.loginfo("Subscribe to topic carry .....")
		rospy.sleep(2)
		######################
		# Define five poses

		self.joint1.publish(init_pose[0])
		rospy.sleep(2)
		self.joint2.publish(init_pose[1])
		rospy.sleep(2)
		self.joint3.publish(init_pose[2])
		rospy.sleep(2)
		self.joint4.publish(init_pose[3])
		rospy.sleep(2)
		self.joint5.publish(init_pose[4])
		rospy.sleep(2)

		rospy.loginfo("Connecting to turtlebot arm....")
	
		while not rospy.is_shutdown():

	#rospy.loginfo("Test ......")
			if self.mark == 1:
				# Rotate joint one to get the direction
				self.joint1.publish(hold_pose[0])
				rospy.sleep(2)
				self.joint2.publish(hold_pose[1])
				rospy.sleep(2)
				self.joint3.publish(hold_pose[2])
				rospy.sleep(2)
				self.joint4.publish(hold_pose[3])
				rospy.sleep(2)
				self.joint5.publish(hold_pose[4])
				rospy.sleep(2)
				self.mark = 0

			if self.mark == 2:
				self.joint5.publish(init_pose[4])
				rospy.sleep(2)
				self.joint4.publish(init_pose[3])
				rospy.sleep(2)
				self.joint3.publish(init_pose[2])
				rospy.sleep(2)
				self.joint2.publish(init_pose[1])
				rospy.sleep(2)
				self.joint1.publish(init_pose[0])
				rospy.sleep(2)

				self.joint1.publish(set_pose[0])
				rospy.sleep(2)
				self.joint2.publish(set_pose[1])
				rospy.sleep(2)
				self.joint4.publish(set_pose[3])
				rospy.sleep(2)
				self.joint3.publish(set_pose[2])
				rospy.sleep(2)
				self.joint5.publish(set_pose[4])
				rospy.sleep(2)
				self.mark = 0
				self.ifreleased = "find_person"
				self.released_pub.publish(self.ifreleased)
				
	def cleanup(self):
		rospy.loginfo("Shutting down turtlebot arm....")

if __name__=="__main__":

	rospy.init_node('arm')
	try:
		Loop()
		rospy.spin()
	except rospy.ROSInterruptException:
			pass

