#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation')

import rospy
import serial

from std_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan

'''
percept interprets the laserscan for maze characterization
'''


def main():
    global pub
    rospy.loginfo("starting laserscanning: perception...")
    rospy.init_node('percept')

    #pub = rospy.Publisher('/cmd_vel', Twist) 

    rospy.Subscriber("/scan", LaserScan, scanned) 
    rospy.spin()

def scanned(data):
    string = ""
    accumulatedRange = 0
    for i in range (170, 190): # centered about 180
        dist = data.ranges[i]
        if dist > 10:
            dist = 0
        string += str(dist) + " "

        accumulatedRange+=dist

    rospy.loginfo(accumulatedRange/20)
    #rospy.loginfo(data)



if __name__ == "__main__":
    main()
