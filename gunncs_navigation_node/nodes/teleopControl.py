#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

'''
teleopControl.py sends Twist commands from a joystick.
'''


def main():
    global pub
    rospy.loginfo("starting Twist Publishing via Joystick...")
    rospy.init_node('teleopControl')

    pub = rospy.Publisher('/cmd_vel', Twist) 
    rospy.Subscriber("/joy", Joy, joystickChanged) 
    rospy.spin()

def joystickChanged(data):
    #print data.axes[1]
    #print data.axes[5]
    msg = Twist()
    msg.linear.x = .5*data.axes[1]
    msg.angular.z = 2*data.axes[0] 
    pub.publish(msg)





if __name__ == "__main__":
    main()
