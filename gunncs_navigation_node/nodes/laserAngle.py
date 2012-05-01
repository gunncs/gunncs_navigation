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

from pyqtgraph.Qt import QtGui, QtCore
from numpy import arange, array, ones, linalg
import numpy as np
import pyqtgraph as pg

from pylab import plot, show


INTERSECT_THRESHOLD = .04
NEIGHBOR_RANGE= 1
COLORS = {}

def main():
    rospy.loginfo("starting laserAngle")
    #global window, win, plot, rangeCurve
    #open plotter
    rospy.loginfo("starting plotter...")


    ''' 
    PLOTS
    '''

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
    errorPlot.setXRange(120, 250)
    errorCurve = errorPlot.plot(pen=(255, 0,0))
    centersCurve = errorPlot.plot(pen=(0,255,0))


    '''
    Visualization
    '''


    #open vis
    rospy.loginfo("starting visualization...")
    global window
    pygame.init()
    window = pygame.display.set_mode((640,640))

    rospy.loginfo("subscribing to laser...")
    rospy.init_node('laserAngle')
    rospy.Subscriber("/scan", LaserScan, scanned) 
    app.exec_()
    rospy.spin()

def scanned(laserscan):
    '''
    laserscan callback
    '''
    global  window, win, rangeCurve

    #update plot
    rangeCurve.setData(np.array(laserscan.ranges))

    #draw vis
    errors = getErrorMap(laserscan)
    intersects = getIntersects(errors)
    #print(intersects)
    errorClusters= getClusters(intersects)
    #print errorClusters
    drawVis(laserscan, errorClusters)
    #print(intersects)

def getClusters(positions):
    '''
    given a list of points, points are clustered.
    we return the positions of the centers of these clusters.
    '''

    clusters= [] #to return
    currentClusterStart = -1 #we start without a cluster
    previousPos = positions[0] -1 # we force the first item to be a cluster


    #print positions
    for position in positions:
        '''
        print("pos:" + str(position) + "\t" + "cs" + str(currentClusterStart) + "\tpp:" + str(previousPos))
        '''
        #if we see that we are continuous,
        #if previousPos + 1 is position:
        if position - previousPos is 1:
            #and this is the first time, we record this position.
            if currentClusterStart is -1:
                currentClusterStart= previousPos
        #if we lost continuity
        else:
            #we end the cluster
            clusters.append([currentClusterStart , previousPos])
            # and reset
            currentClusterStart = -1
                
        previousPos = position
    #add the last cluster set, that isn't iterated
    clusters.append([currentClusterStart , previousPos])

    return clusters

def getIntersects(errors):
    global centersCurve, INTERSECT_THRESHOLD

    positions = [] 
    outOfThreshold = [] 
    #INTERSECT_THRESHOLD if not within threshold, 0 otherwise
    for i in range(0, len(errors)):
        if errors[i] > INTERSECT_THRESHOLD:
            #positions.append(i)
            outOfThreshold.append(INTERSECT_THRESHOLD)
        else:
            positions.append(i)
            outOfThreshold.append(0)

    #print(positions)

    centersCurve.setData(np.array(outOfThreshold))

    return positions


def deltaR(laserscan, id, neighborRange):
    '''
    calculates the change of R-- radius-- from
    surrounding points indicated by the range
    to the point at id
    '''
    mydistance = laserscan.ranges[id]
    error = 0
    lowerbound = (id - neighborRange) % len(laserscan.ranges)
    upperbound = (id + neighborRange) % len(laserscan.ranges)
    for i in range(lowerbound, upperbound):
        #we take an absolute value so errors don't cancel out
        error += math.fabs(mydistance - laserscan.ranges[i])

    #normalize output
    return math.fabs(error / ((2 * neighborRange) + 1))
        

def filter(laserscan):
    global NEIGHBOR_RANGE
    corrections = list()
    ranges = list()
    for i in range(0, len(laserscan.ranges)-1):
        ranges.append(laserscan.ranges[i])
        error = deltaR(laserscan, i, NEIGHBOR_RANGE)
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

        
def getErrorMap(laserscan):
    #print laserscan
    global errorCurve, NEIGHBOR_RANGE
    #print(calculateSlope(laserscan, 200, 210))
    errors= []
    for i in range(0, len(laserscan.ranges)):
        errors.append(deltaR(laserscan, i, NEIGHBOR_RANGE))

    errorCurve.setData(np.array(errors))
    return errors

        #print(str(p1) + "\t" + str(p1[0]))
    

def drawVis(laserscan, bounds):
    '''
    Draws the laserscan 
    currently the points are colorized according to 
    the clusters identified.

    '''
    global window
    pygame.draw.rect(window, (0,0,0), (0, 0, 640, 640))
    pygame.draw.circle(window, (255,0,0), (320, 640), 20, 0)

    
    #for i in range( 0, len(bounds)):
        #print getRegression(laserscan, bounds[i])
    #print "-------------------"

    currentBound= 0 #which bound we are in
    color = nonRandomColor(bound) # color for the bound
    slope = 0
    
    for i in range(0, len(laserscan.ranges)):
        #advance bound if out of bounds
        theta = -(laserscan.angle_min + laserscan.angle_increment * i)
        x = 320 + 300 *laserscan.ranges[i] * math.sin(theta)
        y = 640 - 300 *laserscan.ranges[i] * math.cos(theta)

        if i is (bounds[currentBound % len(bounds)])[0]:
            #print("advancing: " + str((bounds[bound % len(bounds)])[0]))
            color = nonRandomColor(currentBound)
            slope, intercept = getRegression(laserscan, bounds[currentBound% len(bounds)])

            j = (bounds[currentBound % len(bounds)])[1]

            thetaf = -(laserscan.angle_min + laserscan.angle_increment * j)
            xf = 320 + 300 *laserscan.ranges[j] * math.sin(thetaf)
            dx = xf - x

            pygame.draw.circle(window, (255,0,0), (int(x), int(y)), 4, 0)
            pygame.draw.line(window, color, (int(x), int(y)), (int(xf ), int(y - dx*slope))) 
            currentBound = currentBound + 1
        else:
            #dont know why we need the negative sign...
            pygame.draw.circle(window, color, (int(x), int(y)), 1, 0)

    pygame.display.flip()

def getRegression(laserscan, currentBound):
    '''
    Given a laserscan and relevant bounds, calculates 
    the cartesian slope of the feature using
    linear regression
    '''
    tmpx = []
    tmpy = []

    for i in range(bound[0], bound[1]):
        #calculates cartesian coordinates
        theta = -(laserscan.angle_min + laserscan.angle_increment * i)
        x = laserscan.ranges[i] * math.sin(theta)
        y = laserscan.ranges[i] * math.cos(theta)

        #store coordinates
        tmpx.append(x)
        tmpy.append(y)

    x = np.array(tmpx)
    A = np.array([x, ones(len(tmpx))])
    y = np.array(tmpy)

    #perform regression
    w = linalg.lstsq(A.T, y)[0]

    '''
    line = w[0]*x + w[1]
    plot(x, line, 'r-', x,y,'o')
    show()

    '''
    #print 'r value', r_value
    #print  'p_value', p_value
    #print 'standard deviation', std_err

    #line = slope*xi+intercept
    #plot(x,line,'r-',x,y,'o')
    #show()
    return w
    
    



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


if __name__ == "__main__":
    main()
