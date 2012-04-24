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


NEIGHBOR_RANGE= 2

def main():
    rospy.loginfo("starting laserAngle")
    #global window, win, plot, rangeCurve
    #open plotter
    rospy.loginfo("starting plotter...")

    global rangeCurve, errorCurve, centersCurve
    app = QtGui.QApplication([])
    
    win = pg.GraphicsWindow(title="Basic plotting examples")
    win.resize(800,600)

    rangePlot = win.addPlot(title="Ranges")
    rangePlot.enableAutoRange('xy', True)
    rangeCurve = rangePlot.plot(pen='y')

    errorPlot = win.addPlot(title="delta R")
    #errorPlot.enableAutoRange('xy', True)
    #errorPlot.setYRange(0, .05)
    errorPlot.setYRange(0, .2)
    errorPlot.setXRange(120, 230)
    errorCurve = errorPlot.plot(pen=(255, 0,0))
    centersCurve = errorPlot.plot(pen=(0,255,0))


    #open vis
    rospy.loginfo("starting visualization...")
    global window
    pygame.init()
    window = pygame.display.set_mode((640,480))

    rospy.loginfo("subscribing to laser...")
    rospy.init_node('laserAngle')

    rospy.Subscriber("/scan", LaserScan, scanned) 
    app.exec_()
    rospy.spin()

def scanned(laserscan):
    global  window, win, rangeCurve

    #laserscan = filter(laserscan)
    #laserscan= filter(laserscan)
    #laserscan= filter(laserscan)

    #update plot
    rangeCurve.setData(np.array(laserscan.ranges))

    #draw vis
    drawVis(laserscan)
    errors = calculateError(laserscan)
    filterError(errors)

def filterError(errors):

    global centersCurve

    positions = []
    values = []
    for i in range(0, len(errors)):
        if errors[i] > .04:
            positions.append(i)
            values.append(.03)
        else:
            positions.append(i)
            values.append(0)

    #print(positions)

    centersCurve.setData(np.array(positions), np.array(values))


def getNeighborDistance(laserscan, id, neighborRange):
    mydistance = laserscan.ranges[id]
    error = 0
    lowerbound = (id - neighborRange) % len(laserscan.ranges)
    upperbound = (id + neighborRange) % len(laserscan.ranges)
    for i in range(lowerbound, upperbound):
        error += mydistance - laserscan.ranges[i]

    return math.fabs(error / ((2 * neighborRange) + 1))
        

def filter(laserscan):
    global NEIGHBOR_RANGE
    corrections = list()
    ranges = list()
    for i in range(0, len(laserscan.ranges)-1):
        ranges.append(laserscan.ranges[i])
        error = getNeighborDistance(laserscan, i, NEIGHBOR_RANGE)
        if error > .04:
            #toadd = [i, laserscan.ranges[i + 1)]
            corrections.append((i, laserscan.ranges[i+1]))
            #corrections.append(toadd)
        


    for i in range(0, len(corrections) ):
        #laserscan.ranges[(corrections[i])[0]] = (corrections[i])[1]
        ranges.pop((corrections[i])[0])
        ranges.insert(i, (corrections[i])[1])
    

    laserscan.ranges = ranges;
    return laserscan

        
def calculateError(laserscan):
    #print laserscan
    global errorCurve, NEIGHBOR_RANGE
    #print(calculateSlope(laserscan, 200, 210))

    
    errors= []

    for i in range(0, len(laserscan.ranges)):
        errors.append(getNeighborDistance(laserscan, i, NEIGHBOR_RANGE))

    errorCurve.setData(np.array(errors))
    return errors
    


        #print(str(p1) + "\t" + str(p1[0]))
def calculateSlope(laserscan, n1, n2):
    p1 = getPoint(laserscan, n1)
    p2 = getPoint(laserscan, n2)
    m = (p2[1] - p1[1]) / (p2[0] - p1[0])
    return m

    

def getPoint(laserscan, n):
    theta = laserscan.angle_min + laserscan.angle_increment * n
    x = laserscan.ranges[n] * math.cos(theta)
    y = laserscan.ranges[n] * math.sin(theta)
    return (x, y)

def drawVis(laserscan):
    global window
    pygame.draw.rect(window, (0,0,0), (0, 0, 640, 480))
    pygame.draw.circle(window, (255,0,0), (320, 240), 20, 0)

    for i in range(0, len(laserscan.ranges)):
        #dont know why we need the negative sign...
        theta = -(laserscan.angle_min + laserscan.angle_increment * i)
        x = 320 + 100 *laserscan.ranges[i] * math.cos(theta)

        y = 240 + 100 *laserscan.ranges[i] * math.sin(theta)
        
        pygame.draw.circle(window, (0,255,0), (int(x), int(y)), 1, 0)

    pygame.display.flip()


if __name__ == "__main__":
    main()
