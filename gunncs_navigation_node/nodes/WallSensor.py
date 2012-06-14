#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import math
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
    global anglePub, distancePub

    #first determine relevant wall
    rightWall = getWallClusterId(laserScan)

    #get the angle of the wall
    angle = getWallAngle(laserScan, rightWall)
    angleMsg = Float32()
    angleMsg.data = angle
    anglePub.publish(angleMsg)
    
    #get distance
    distance = getWallDistance(laserScan, rightWall)
    distMsg = Float32()
    distMsg.data = distance
    distancePub.publish(distMsg)

def getWallClusterId(laserScan):
    '''
    determines which feature is the wall we desire,
    and returns its id
    '''
    return 1

def getWallAngle(laserscan, wallID):
    '''
    examines the wall feature and determines its  angle,
    where 0 is parallel. turning left is a positive angle,
    moving right is a negative angle
    '''
    
    slope = laserscan.features[wallID].regression.slope
    return math.atan(slope)*180/math.pi+90


def getWallDistance(laserscan, wallID):
    '''
    examines the wall feature and determines its angle
    to the robot. 

    '''
    regression = laserscan.features[wallID].regression
     
    return abs(regression.y_intercept)/math.sqrt(regression.slope*regression.slope+1)




if __name__ == "__main__":
    main()
