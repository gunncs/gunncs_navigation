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


from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg


def main():
    rospy.loginfo("starting laserAngle")
    global pub, window, win, plot, curve
    #open plotter
    rospy.loginfo("starting plotter...")
    app = QtGui.QApplication([])
    
    win = pg.GraphicsWindow(title="Basic plotting examples")
    win.resize(800,600)

    plot = win.addPlot(title="Ranges")
    plot.enableAutoRange('xy', True)
    curve = plot.plot(pen='y')


    #open vis
    rospy.loginfo("starting visualization...")
    pygame.init()
    window = pygame.display.set_mode((640,480))

    rospy.loginfo("subscribing to laser...")
    rospy.init_node('laserAngle')

    rospy.Subscriber("/scan", LaserScan, scanned) 
    app.exec_()
    rospy.spin()

def scanned(data):
    global pub, window, win, plot, curve

    #update plot
    curve.setData(np.array(data.ranges))

    #draw vis
    pygame.draw.rect(window, (0,0,0), (0, 0, 640, 480))
    pygame.draw.circle(window, (255,0,0), (320, 240), 20, 0)

    for i in range(0, len(data.ranges)):
        theta = data.angle_min + data.angle_increment * i
        x = 320 + 100 *data.ranges[i] * math.cos(theta)

        y = 240 + 100 *data.ranges[i] * math.sin(theta)
        
        pygame.draw.circle(window, (0,255,0), (int(x), int(y)), 1, 0)

    pygame.display.flip()




if __name__ == "__main__":
    main()
