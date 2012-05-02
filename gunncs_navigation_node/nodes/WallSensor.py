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

    anglePub = rospy.Publisher('/wall_angle', Float32) 
    distancePub = rospy.Publisher('/wall_distance', Float32) 

    rospy.Subscriber("/featured_scan", LineFeaturedLaserScan, laserScanned)

    # run ros loop, hangs until process killed
    rospy.spin()

def laserScanned(laserScanned):
    print(laserScanned)


if __name__ == "__main__":
    main()
