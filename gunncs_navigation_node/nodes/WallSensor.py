#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gunncs_navigation_msgs.msg import *

'''
WallSensor receives LineFeaturedLaserScan messages and publishes 
the angle of the right side wall, as well as distance.

'''


def main():
    global anglePub, distancePub
    rospy.loginfo("starting WallSensor...")
    rospy.init_node('WallSensor')

    # we are publishing angle and distance
    anglePub = rospy.Publisher('/wall_angle', Float32) 
    distancePub = rospy.Publisher('/wall_distance', Float32) 

    # we process a featured laser scan
    rospy.Subscriber("/featured_scan", FeaturedLaserScan, laserScanned)

    # run ros loop, hangs until process killed
    rospy.spin()

def laserScanned(laserScan):
    '''
    callback, when a LineFeaturedLaserScan is published
    '''

    #first determine relevant wall
    rightWall = getWallClusterId(laserScan)

    #get the angle of the wall
    angle = getWallAngle(laserScan, rightWall)

    #get distance
    distance = getWallDistance(laserScan, rightWall)

    print(laserScanned)


def getWallClusterId(laserScan):
    '''
    determines which feature is the wall we desire,
    and returns its id
    '''
    return 0

def getWallAngle(laserscan, wallID):
    '''
    examines the wall feature and determines its  angle,
    where 0 is parallel. turning left is a positive angle,
    moving right is a negative angle
    '''
    return 0


def getWallDistance(laserscan, wallID):
    '''
    examines the wall feature and determines its angle
    to the robot. 

    '''
    return 0




if __name__ == "__main__":
    main()
