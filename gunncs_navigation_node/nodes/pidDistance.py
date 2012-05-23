#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial

from std_msgs.msg import *
from sensor_msgs.msg import LaserScan


'''
percept interprets the laserscan for maze characterization
'''

def main():
    global pub
    rospy.loginfo("starting laserscanning: perception...")
    rospy.init_node('percept')

    pub = rospy.Publisher('/distance', Float32) 

    rospy.Subscriber("/scan", LaserScan, scanned) 
    rospy.spin()

def scanned(data):
    global pub
    distance = getDistance(data)

    pub.publish(distance);
    
    rospy.loginfo("distance;" + str(distance))

def command(pidResult):
    global pub, LINEAR_SPEED
    msg = Twist()
    msg.linear.x = LINEAR_SPEED
    msg.angular.z = pidResult
    pub.publish(msg)


def getDistance(laserscan):
    '''
    Takes an average of the distance about the 180th range

    '''
    minDistance = 11
    index = 0
    for i in range(0,360):
        if laserscan.ranges[i] < minDistance:
            index = i
            minDistance = laserscan.ranges[i]


    '''
    string = ""
    accumulatedRange = 0
    for i in range (index -5, index + 5): # centered about minimum
        dist = laserscan.ranges[i]
        if dist > 10:
            dist = 0
        string += str(dist) + " "

        accumulatedRange+=dist

    return accumulatedRange/20
    '''
    return minDistance


if __name__ == "__main__":
    main()
