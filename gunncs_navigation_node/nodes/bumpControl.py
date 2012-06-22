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
DIST_BT_CELLS = 1
DIST_CELL_CENTER_TO_WALL = 1
LINEAR_SPEED = 0.5
TURNWISE_SPEED = 0.5
SLEEPYTIME = 0.01

'''
Publishes an odom frame im meters and degrees '''
def poseChange(data):
    global x, y, theta
    x = data.x
    y = data.y
    theta = data.theta

def moveForward(requestedDistance): #returns false if bump sensor is triggered
    global x, y,bumped
    oldx = x
    oldy = y
    distance = 0
    msg = Twist()
    msg.linear.x = LINEAR_SPEED
    pub.publish(msg)
    while (distance < requestedDistance):
        pub.publish(msg)
        distance = math.sqrt( (oldx - x) * (oldx - x) + (oldy - y) * (oldy - y))
        rospy.sleep(SLEEPYTIME)
        if (bumped):
            msg.linear.x = 0
            pub.publish(msg)
            return False
    msg.linear.x = 0
    pub.publish(msg)
    return True

def moveBackward(requestedDistance):
    global x, y
    oldx = x
    oldy = y
    distance = 0
    msg = Twist()
    msg.linear.x = -LINEAR_SPEED
    while (distance < requestedDistance):
        pub.publish(msg)
        distance = math.sqrt( (oldx - x) * (oldx - x) + (oldy - y) * (oldy - y))
        rospy.sleep(SLEEPYTIME)
    msg.linear.x = 0
    pub.publish(msg)

def investigateWall(): #returns false if we returned to our current cell
    bumped = not moveForward(DIST_BT_CELLS)
    if (bumped): 
        moveBackward(DIST_CELL_CENTER_TO_WALL)
    return bumped

def botStateChanged(data):
    global bumped, commanded
    bumped = data.bumps_wheeldrops is not 0

def turnRight():
    global theta, pub
    originalTheta = theta;
    msg = Twist()
    msg.angular.z = -TURNWISE_SPEED
    pub.publish(msg)
    difference = (originalTheta - theta + 360 )%180
    while (difference< 90):
        rospy.loginfo(difference)
        difference = (originalTheta - theta + 360 )%180
        pub.publish(msg)
        rospy.sleep(SLEEPYTIME)
    msg.angular.z = 0
    pub.publish(msg)


def turnLeft():
    global theta, pub
    originalTheta = theta;
    msg = Twist()
    msg.angular.z = TURNWISE_SPEED
    pub.publish(msg)
    difference = (theta - originalTheta + 360 )%180
    while (difference <90):
        rospy.loginfo(difference)
        difference = (theta - originalTheta + 360 )%180
        pub.publish(msg)
        rospy.sleep(SLEEPYTIME)
    msg.angular.z = 0
    pub.publish(msg)



def main():
    global pub, theta, x, y, bumped, commanded
    theta = 0
    x = 0
    y = 0
    bumped = False
    commanded = False
    rospy.loginfo("converting quaternions")
    rospy.init_node('bumpControl')
    rospy.Subscriber("/odom2D", Pose2D, poseChange)
    rospy.Subscriber("/turtlebot_node/sensor_state", TurtlebotSensorState, botStateChanged)
    #rospy.Subscriber("/cmd_vel", Twist, twistChanged)
    pub = rospy.Publisher("/cmd_vel", Twist)

    '''
    main logic:
    always try to take right turns, then forward direction, then left.
    '''
    while theta is 0:
        rospy.loginfo("waiting to receive odom...")
        rospy.sleep(0.01)

    result = moveBackward(.5)
    result = moveForward(.5)
    while not rospy.is_shutdown():
        #rospy.loginfo(bumped)
        rospy.sleep(0.01)
        #turn right
        #if bump, return back to old cell
        #



main()
