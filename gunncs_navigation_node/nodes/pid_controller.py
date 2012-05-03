#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation')
import rospy
import signal
import sys
import math

class PIDController:

    def __init__(self, p, i, d, minOutput, maxOutput):
        self.kP = p
        self.kI = i
        self.kD = d
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.errorAccumulated = 0
        self.error = 0
        self.errorDelta = 0
        self.errorPrevious = 0
        self.maxITerm = .1;
        self.prevTime = 0
        self.result = 0


    def setPID(self, p, i, d):
        self.kP = p
        self.kI = i
        self.kD = d

    def setOutputRange(self, minOutput, maxOutput):
        self.minOutput = minOutput
        self.maxOutput = maxOutput

    def setIMax(self, maxITerm):
        self.maxITerm = maxITerm

    def update(self, current, target):
        global MS_TO_SECONDS
        
        time = rospy.get_time()
        if self.prevTime < 0:
            self.prevTime = time

        #rospy.loginfo("time: " + str(time))
        deltaTime = time - self.prevTime

        self.error = target - current
        self.errorAccumulated = self.errorAccumulated + self.error * deltaTime
        self.errorDelta = (self.error - self.errorPrevious) / deltaTime
        self.errorPrevious = self.error
        self.prevTime = time

        if math.fabs(self.errorAccumulated * self.kI) > self.maxITerm:
            self.errorAccumulated = self.maxITerm / self.kI * sigNum(self.errorAccumulated)

        self.calculateResult()


    def calculateResult(self):
        self.result = self.kP * self.error + self.kI * self.errorAccumulated + self.kD * self.errorDelta

    def getResult(self):
        return self.result
        
def sigNum(d):
    if d > 0:
        return 1
    if d < 0:
        return -1
    return 0


if __name__ == '__main__':
    print "pid controller"
    print sigNum(2321)
    print sigNum(-21)


    

