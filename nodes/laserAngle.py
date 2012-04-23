#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation')

import rospy
import serial
import sys
import pygame
import math

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *



def main():
    global pub, window 
    pygame.init()
    window = pygame.display.set_mode((640,480))

    rospy.loginfo("starting laserAngle")
    rospy.init_node('laserAngle')

    rospy.Subscriber("/scan", LaserScan, scanned) 
    rospy.spin()

def scanned(data):
    global window
    pygame.draw.rect(window, (0,0,0), (0, 0, 640, 480))
    pygame.draw.circle(window, (255,0,0), (320, 240), 20, 0)

    for i in range(0, len(data.ranges)):
        x = 320 + 100 *data.ranges[i] * math.cos(data.angle_min + (data.angle_increment * i));

        y = 240 + 100 *data.ranges[i] * math.sin(data.angle_min + (data.angle_increment * i));
        
        pygame.draw.circle(window, (0,255,0), (int(x), int(y)), 1, 0)

    pygame.display.flip()




if __name__ == "__main__":
    main()
