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
DIST_BT_CELLS = .4
DIST_CELL_CENTER_TO_WALL = .08
LINEAR_SPEED = 0.5
TURNWISE_SPEED = 1
SLEEPYTIME = 0.001
WALL_UNKNOWN = 1
WALL_BLOCKED = 1
WALL_OPEN = 2

def explore(): #do right wall following while generating a map
    horizontalWalls = [[0 for x in range(40)]for x in range(40)]
    verticalWalls = [[0 for x in range(40)]for x in range(40)]
    currentCellX = 20
    currentCellY = 20
    virtualTheta = 0 #0,90,180,270 are the only allowed values
    finished = False
    didBump = False
    while (not finished):
        newCell= investigateDirection()
        rospy.sleep(2)
        if (newCell):
            '''
            if (virtualTheta == 0):
                horizontalWalls[currentCellX][currentCellY] = WALL_OPEN
                currentCellY-=1
            elif (virtualTheta == 90):
                verticalWalls[currentCellX][currentCellY] = WALL_OPEN
                currentCellX-=1
            elif (virtualTheta == 180):
                horizontalWalls[currentCellX][currentCellY+1] = WALL_OPEN
                currentCellY+=1
            elif (virtualTheta ==  270):
                verticalWalls[currentCellX+1][currentCellY] = WALL_OPEN
                currentCellX+=1
            virtualTheta+=270;virtualTheta%=360
            '''
            turnRight()
        else:
            '''
            if (virtualTheta ==  0):
                horizontalWalls[currentCellX][currentCellY] = WALL_BLOCKED
                currentCellY-=1
            elif (virtualTheta ==  90):
                verticalWalls[currentCellX][currentCellY] = WALL_BLOCKED
                currentCellX-=1
            elif (virtualTheta ==  180):
                horizontalWalls[currentCellX][currentCellY+1] = WALL_BLOCKED
                currentCellY+=1
            elif (virtualTheta ==  270):
                verticalWalls[currentCellX+1][currentCellY] = WALL_BLOCKED
                currentCellX+=1
            virtualTheta+=270;virtualTheta%=360
            '''
            turnLeft()


'''
Publishes an odom frame im meters and degrees '''
def poseChange(data):
    global x, y, theta
    x = data.x
    y = data.y
    theta = data.theta

def moveForward(requestedDistance): #returns False if bump sensor is triggered
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

def investigateDirection(): #returns True if we explore a new cell
    print "investigating direction...."
    global angleCorrection
    angleCorrection = 0
    print "moving forward"
    bumped = not moveForward(DIST_BT_CELLS)
    if (bumped): 
        print "bumped: moving backward"
        moveBackward(DIST_CELL_CENTER_TO_WALL)
        print ("angleCorrection:" + str(angleCorrection))
        if (angleCorrection != 0): #indicating we are tilted
            #perform a turn
            turnDegrees(5 * angleCorrection)
    return not bumped #not bumped means we got no obstacles, meaning new cell

def botStateChanged(data):
    global bumped, commanded, angleCorrection
    bumped = data.bumps_wheeldrops is not 0
    if (data.bumps_wheeldrops == 1):
        angleCorrection = -1
        print "setting correction to go left"
    elif (data.bumps_wheeldrops ==2):
        angleCorrection = 1
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



def turnRight():
    turnDegrees(-90)


def turnLeft():
    turnDegrees(90)



def main():
    global pub, theta, x, y, bumped, commanded
    theta = -999
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
    while theta is -999 and not rospy.is_shutdown():
        rospy.loginfo("waiting to receive odom...")
        rospy.sleep(0.01)
    horizontalWalls, verticalWalls = explore()


main()
