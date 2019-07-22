#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/03/07
    Author: Xu Yucheng
    Abstract: control arm with keyboard
"""

import os
import sys
import time
import requests
import json
import hashlib
import base64
import rospy
import sys, select, termios, tty
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from std_msgs.msg import UInt32

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class arm_keyboard():
    def __init__(self):
        self.joint1 = rospy.Publisher('/arm_shoulder_pan_joint/command',Float64, queue_size=1)
        self.joint2 = rospy.Publisher('/arm_shoulder_lift_joint/command',Float64, queue_size=1)
        self.joint3 = rospy.Publisher('/arm_elbow_flex_joint/command',Float64, queue_size=1)
        self.joint4 = rospy.Publisher('/arm_wrist_flex_joint/command',Float64, queue_size=1)
        self.joint5 = rospy.Publisher('/gripper_joint/command',Float64, queue_size=1)
        self.msg = """
                    Control Your Turtlebot arm!
                    ---------------------------
                    Moving around:
                    t    y    u    i    o
                    g    h    j    k    l
                    CTRL-C to quit
                    """
        self.joint1_curr = 0
        self.joint2_curr = 0
        self.joint3_curr = 0
        self.joint4_curr = 0
        self.joint5_curr = 0

        print (self.msg)
        self.keyboard_control()
    def keyboard_control(self):
        while(1):
            key = getKey()
            if key == 't':
                self.joint1_curr += 1
                self.joint1.publish(0.05*self.joint1_curr)
            if key == 'g':
                self.joint1_curr -= 1
                self.joint1.publish(0.05*self.joint1_curr)
            if key == 'y':
                self.joint2_curr += 1
                self.joint2.publish(0.05*self.joint2_curr)
            if key == 'h':
                self.joint2_curr -= 1
                self.joint2.publish(0.05*self.joint2_curr)
            if key == 'u':
                self.joint3_curr += 1
                self.joint3.publish(0.05*self.joint3_curr)
            if key == 'j':
                self.joint3_curr -= 1
                self.joint3.publish(0.05*self.joint3_curr)
            if key == 'i':
                self.joint4_curr += 1
                self.joint4.publish(0.05*self.joint4_curr)
            if key == 'k':
                self.joint4_curr -= 1
                self.joint4.publish(0.05*self.joint4_curr)
            if key == 'o':
                self.joint5_curr += 1
                self.joint5.publish(0.05*self.joint5_curr)
            if key == 'l':
                self.joint5_curr -=1 
                self.joint5.publish(0.05*self.joint5_curr)
            
if __name__ == '__main__':
    rospy.init_node("arm_keyboard_control")
    ak = arm_keyboard()
    rospy.spin()        
