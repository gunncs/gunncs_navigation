#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial
import sys
import pygame
import math
import random
import time

from geometry_msgs.msg import *

'''
takes command velocities from /safeCmdVel, and if periodically receiving heartbeat signal, relays it to robot
'''

POLL_TIME = .1

ZERO_TWIST = Twist()
ZERO_TWIST.linear.x = 0
ZERO_TWIST.linear.y = 0
ZERO_TWIST.linear.z = 0
ZERO_TWIST.angular.x = 0
ZERO_TWIST.angular.y = 0
ZERO_TWIST.angular.z = 0

def main():
    global pub
    rospy.loginfo("starting emergency stop node")
    rospy.init_node('emergency_stop')
    pub = rospy.Publisher("/cmd_vel", Twist)

    global lastMessageReceivedTime

    rospy.Subscriber("/heartbeat", int64, updateTime)
    rospy.Subscriber("/safeCmdVel", Twist, updateVelocity)


    while not rospy.is_shutdown():
        if time.time() - lastMessageReceivedTime > POLL_TIME:
            stopRobot()
        rospy.sleep(POLL_TIME)

def updateTime(msg):
    global lastMessageReceivedTime
    lastMessageReceivedTime = time.time()

def updateVelocity(safeCmdVel):
    global lastMessageReceivedTime
    #if last message received was too long ago
    if time.time() - lastMessageReceivedTime > POLL_TIME:
        stopRobot() #stop robot
    else: #last message received was recent enough
        pub.publish(safeCmdVel) #robot drives

def stopRobot():
    global pub
    pub.publish(ZERO_TWIST)

main()
