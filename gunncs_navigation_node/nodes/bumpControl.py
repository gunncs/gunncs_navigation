#!/usr/bin/env python
import roslib
roslib.load_manifest('gunncs_navigation_node')
roslib.load_manifest('turtlebot_node')
roslib.load_manifest('tf')
import tf
import rospy
import math
from nav_msgs.msg import *
from geometry_msgs.msg import *
from turtlebot_node.msg import *

'''
Publishes an odom frame im meters and degrees
'''
def poseChange(data):
    global x, y, theta
    x = data.x
    y = data.y
    theta = data.theta


def botStateChanged(data):
    global bumped
    bumped = data.distance ==0

def turnRight():
    global theta, pub
    originalTheta = theta;
    msg = Twist()
    msg.angular.z = -0.5
    pub.publish(msg)
    difference = (originalTheta - theta + 360 )%180
    while (difference< 90):
	rospy.loginfo(difference)
	difference = (originalTheta - theta + 360 )%180
        pub.publish(msg)
        rospy.sleep(0.01)
    msg.angular.z = 0
    pub.publish(msg)
    
    
def turnLeft():
    global theta, pub
    originalTheta = theta;
    msg = Twist()
    msg.angular.z = 0.5
    pub.publish(msg)
    while (theta - originalTheta < 90):
	print "lol"



def main():
    global pub, theta, x, y
    theta = 0
    x = 0
    y = 0
    rospy.loginfo("converting quaternions")
    rospy.init_node('bumpControl')
    rospy.Subscriber("/odom2D", Pose2D, poseChange)
    rospy.Subscriber("/turtlebot_node/sensor_state", TurtlebotSensorState, botStateChanged)
    pub = rospy.Publisher("/cmd_vel", Twist)
    
    '''
    main logic:
    always try to take right turns, then forward direction, then left.
    '''
    while theta is 0:
	rospy.loginfo("waiting to receive odom...")
	rospy.sleep(0.01)
	
    turnRight()
    while not rospy.is_shutdown():
        rospy.loginfo(theta)
        rospy.sleep(0.01)
        #turn right
        #if bump, return back to old cell
        #
        


main()