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
WallHugger.py commands a desired wall angle given the 
wall distance
'''

SETPOINT = .75

KP = 1 
KI = 0 
KD = 0 

pid = PIDController(KP, KI, KD, -1.0, 1.0)

def main():
    global pub
    rospy.loginfo("starting WallHugger...")
    rospy.init_node('WallHugger')

    anglePub = rospy.Publisher('/cmd_wall_angle', Float32) 

    rospy.Subscriber("/wall_distance", Float32, distanceCb) 
    rospy.spin()

def distanceCb(distance):
    '''
    ros callback for wall distance, measured by WallSensor
    '''
    
    global pid, SETPOINT
    pid.update(distance.data, SETPOINT)


def command(pidresult):
    '''
    commands wall angle with the pid result
    '''
    global anglePub
    msg = Float32()
    msg.data = pidresult
    anglePub.publish(msg)
    




if __name__ == "__main__":
    main()
