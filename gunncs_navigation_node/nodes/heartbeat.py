#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial
import sys
import pygame
import math
import random
import time

from std_msgs.msg import *
from geometry_msgs.msg import *

'''
provides a .05 second heartbeat when the robot is supposed to be operating
can be enabled or disabled with boolean messages at /robotEnabled
'''

BUTTON_NUM = 1
POLL_TIME = 0.05

def main():
    global pub
    global robotEnabled
    robotEnabled = True
    rospy.loginfo("starting heartbeat")
    rospy.init_node('heartbeat')
    pub = rospy.Publisher("/heartbeat", UInt64)
    rospy.Subscriber("/joy", Joy, joystickChange)

    countNumber = 0

    while (not rospy.is_shutdown()) and (robotEnabled == True):
        pub.publish(countNumber)
        countNumber = countNumber + 1
        rospy.sleep(POLL_TIME)

def joystickChanged(data):
    global robotEnabled
    if(data.buttons[BUTTON_NUM]):
        robotEnabled = True
    else:
        robotEnabled = False

main()
