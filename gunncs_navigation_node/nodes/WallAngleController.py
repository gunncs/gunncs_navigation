#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gunncs_navigation_msgs.msg import *

'''
WallAngleController.py sends Twist commands from a joystick.
'''


def main():
    global pub
    rospy.loginfo("starting WallAngleController...")
    rospy.init_node('WallAngleController')

    #pub = rospy.Publisher('/cmd_vel', Twist) 
    rospy.Subscriber("/line", Line , lineCB) 
    rospy.spin()

def lineCB(data):
    print(data)


if __name__ == "__main__":
    main()
