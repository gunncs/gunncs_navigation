#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial
import sys
import pygame
import math
import random
import time

'''
provides a .5 second heartbeat when the robot is supposed to be operating
must be stopped manuall
'''

POLL_TIME = 0.5

#provides a 0.5 second heartbeat
def main():
    global pub
    global robotEnabled
    rospy.loginfo("starting dumb heartbeat")
    rospy.init_node('dumbHeartbeat')
    pub = rospy.Publisher("/heartbeat", UInt64)
    rospy.Subscriber("/robotEnabled", Bool, setRobotState)

    countNumber = 0

    while not rospy.is_shutdown():
        pub.publish(countNumber)
        countNumber = countNumber + 1
        rospy.sleep(POLL_TIME)

main()
