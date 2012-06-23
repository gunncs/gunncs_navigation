#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial
import sys
import pygame
import math
import random
import time

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gunncs_navigation_msgs.msg import *


from pyqtgraph.Qt import QtGui, QtCore
from numpy import arange, array, ones, linalg
import numpy as np
import pyqtgraph as pg

COLORS = {}

def main():
    rospy.loginfo("starting laserVisualizer")

    rospy.init_node('laserVisualizer')

    #open vis
    global window
    pygame.init()
    window = pygame.display.set_mode((640,640))

   #subscribe to laser scanner
    rospy.Subscriber("/featured_scan", FeaturedLaserScan, drawVis)

    rospy.spin()

def nonRandomColor(i):
    '''
    accessor for persistent but randomly generated colors
    given an id
    '''
    global COLORS
    if i not in COLORS:
        COLORS[i] = (
                random.randint(0,255),
                random.randint(0,255),
                random.randint(0,255))
    return COLORS[i]

def drawVis(featuredLaserScan):
    '''
    Draws the laserscan
    currently the points are colorized according to
    the clusters identified.

    '''
    global window
    pygame.draw.rect(window, (0,0,0), (0, 0, 640, 640))
    pygame.draw.circle(window, (255,0,0), (320, 320), 20, 0)

    clusters = []
    lines = []
    for feature in featuredLaserScan.features:
        clusters.append([feature.bound.lower, feature.bound.upper])
        lines.append([feature.regression.slope, feature.regression.y_intercept])
    laserscan = featuredLaserScan.laserscan
    currentCluster= 0 #which cluster we are in
    color = nonRandomColor(currentCluster) # color for the cluster
    slope = 0

    for i in range(0, len(laserscan.ranges)):
        #advance cluster if out of clusters
        theta = -(laserscan.angle_min + laserscan.angle_increment * i)
        x = 320 + 300 *laserscan.ranges[i] * math.sin(theta)
        y = 320 - 300 *laserscan.ranges[i] * math.cos(theta)

        #do clusters with points so that the colors are the same
        if i is (clusters[currentCluster % len(clusters)])[0]:
            #print("advancing: " + str((clusters[cluster % len(bounds)])[0]))
            color = nonRandomColor(currentCluster)
            #slope, intercept = getRegression(laserscan, clusters[currentCluster% len(clusters)])
            slope, intercept = lines[currentCluster % len(clusters)]

            j = (clusters[currentCluster % len(clusters)])[1]

            thetaf = -(laserscan.angle_min + laserscan.angle_increment * j)
            xf = 320 + 300 *laserscan.ranges[j] * math.sin(thetaf)
            dx = xf - x

            pygame.draw.circle(window, (255,0,0), (int(x), int(y)), 4, 0)
            pygame.draw.line(window, color, (int(x), int(y)), (int(xf ), int(y - dx*slope)))
            currentCluster = currentCluster + 1
        else:
            #dont know why we need the negative sign...
            pygame.draw.circle(window, color, (int(x), int(y)), 1, 0)

    pygame.display.flip()


if __name__ == "__main__":
    main()
