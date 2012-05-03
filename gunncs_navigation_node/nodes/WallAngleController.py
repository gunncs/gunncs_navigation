#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gunncs_navigation_msgs.msg import *

from pid_controller import *

'''
WallAngleController.py sends Twist commands from a real wall angle
to regulate it to a desired wall angle
'''

KP = 1
KI = 0
KD = 0

pid = PIDController(KP, KI, KD, -1.0, 1.0)

LINEAR_SPEED = .15


def main():
    global pub
    rospy.loginfo("starting WallAngleController...")
    rospy.init_node('WallAngleController')

    twistPub = rospy.Publisher('/cmd_vel', Twist) 

    rospy.Subscriber("/wall_angle", Float32, realAngleCb) 
    rospy.Subscriber("/cmd_wall_angle", Float32, desiredAngleCb) 
    rospy.spin()

def realAngleCb(realAngle):
    '''
    ros callback for wall angle, as measured by a Wall Sensor
    '''
    global pid, setpoint
    pid.update(realAngle.data, setpoint)
    command(pid.getResult())

def desiredAngleCb(angleDesired):
    '''
    ros callback for desired wall angle
    '''
    global setpoint
    setpoint = angleDesired.data

def command(pidResult):
    '''
    sends twist message of the pid result
    '''
    global twistPub, LINEAR_SPEED
    msg = Twist()
    msg.linear.x = LINEAR_SPEED
    msg.angular.z = pidResult
    twistPub.publish(msg)



if __name__ == "__main__":
    main()
