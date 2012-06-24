#!/usr/bin/env python
import roslib
roslib.load_manifest('gunncs_navigation_node')
roslib.load_manifest('turtlebot_node')
roslib.load_manifest('tf')
import tf
import rospy
import math
import pygame
from nav_msgs.msg import *
from geometry_msgs.msg import *
from turtlebot_node.msg import *
from std_msgs.msg import *
SLEEPYTIME = 0.01
TURNWISE_SPEED = 1
def moveForward(requestedDistance):
    global x, y
    oldx = x
    oldy = y
    distance = 0
    msg = Twist()
    msg.linear.x = 1
    while (distance < requestedDistance):
        while bumped:
                msg.angular.z = angleCorrection
                msg.linear.x = -.1
                pub.publish(msg)
                rospy.sleep(SLEEPYTIME)
        msg.angular.z = 0
        msg.linear.x = 1
        pub.publish(msg)
        distance = math.sqrt( (oldx - x) * (oldx - x) + (oldy - y) * (oldy - y))
        rospy.sleep(SLEEPYTIME)
    msg.linear.x = 0
    pub.publish(msg)

def botStateChanged(data):
    global bumped, commanded, angleCorrection
    bumped = data.bumps_wheeldrops is not 0
    if (not bumped):
            angleCorrection = 0
    if (data.bumps_wheeldrops == 1):
        angleCorrection = 1
        print "setting correction to go left"
    elif (data.bumps_wheeldrops ==2):
        angleCorrection = -1
        print "settting correction to go right"

def turnDegrees(degrees):
    global theta, pub
    originalTheta = theta;

    speed = TURNWISE_SPEED
    msg = Twist()
    msg.angular.z = TURNWISE_SPEED * cmp(degrees, 0)
    pub.publish(msg)
    difference = ((originalTheta - theta)* -cmp(degrees, 0) + 360 )%180
    while (difference < abs(degrees * (92.0/90.0))):
        #rospy.loginfo(difference)
        difference = ((originalTheta - theta) * -cmp(degrees, 0) + 360 )%180
        pub.publish(msg)
        rospy.sleep(SLEEPYTIME)
    msg.angular.z = 0
    pub.publish(msg)

def poseChange(data):
    global x, y, theta
    x = data.x
    y = data.y
    theta = data.theta

def main():
    global pub, theta,x,y,bumped
    bumped = False
    theta = -999
    x = 0
    y = 0
    rospy.init_node('bumpControl')
    rospy.Subscriber("/odom2D", Pose2D, poseChange)
    rospy.Subscriber("/turtlebot_node/sensor_state", TurtlebotSensorState, botStateChanged)
    pub = rospy.Publisher("/cmd_vel", Twist)

    '''
    main logic:
    always try to take right turns, then forward direction, then left.
    '''
    while theta is -999 and not rospy.is_shutdown():
        rospy.loginfo("waiting to receive odom...")
        rospy.sleep(0.01)

    input()
    moveForward(100)


if __name__ == "__main__":
    main()

