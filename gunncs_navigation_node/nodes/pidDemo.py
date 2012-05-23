#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial
import signal
import sys
import wx
import math


from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gunncs_navigation_msgs.msg import *

from pid_controller import *

'''
WallHugger.py commands a desired wall angle given the 
wall distance
'''

TUNING = True

SETPOINT = 750

KP = 0
KI = 0 
KD = 0 

pid = PIDController(KP, KI, KD, -1.0, 1.0)

def main():
    global cvPub, twistPub
    rospy.loginfo("starting PIDDemo..")
    rospy.init_node('PIDDemo')

    rospy.Subscriber("/distance", Float32, distanceCb) 
    cvPub = rospy.Publisher('/cv', Float32)
    twistPub = rospy.Publisher('/cmd_vel', Twist)
    if TUNING:
        app = wx.PySimpleApp()
        # create a window/frame, no parent, -1 is default ID, title, size
        frame = wx.Frame(None, -1, "wxSlider Test1", size = (400, 310))
        # call the derived class, -1 is default ID
        MyPanel(frame,-1)
        # show the frame
        frame.Show(True)
        # start the event loop
        app.MainLoop()


    rospy.spin()

def distanceCb(distance):
    '''
    ros callback for wall distance, measured by WallSensor
    '''
    pidUpdate(distance.data)

def pidUpdate(data):
    global pid, SETPOINT

    #print (data)
    pid.update(data, SETPOINT)
    command(pid.getResult())



def command(pidresult):
    '''
    commands wall angle with the pid result
    '''
    global cvPub, twistPub
    cv_msg = Float32()
    cv_msg.data = pidresult
    cvPub.publish(cv_msg)

    twist_msg = Twist()
    twist_msg.linear.x = pidresult
    twistPub.publish(twist_msg)


class MyPanel(wx.Panel):
    """ 
    class MyPanel creates a panel with 2 sliders on it, inherits wx.Panel
    putting your components/widgets on a panel gives additional versatility
    """
    def __init__(self, parent, id):
        # create a panel
        wx.Panel.__init__(self, parent, id) 
        self.SetBackgroundColour("white")

        # wx.SL_VERTICAL  displays the slider vertically
        # wx.SL_HORIZONTAL  displays the slider horizontally
        # wx.SL_AUTOTICKS  displays tick marks
        # wx.SL_LABELS  displays minimum, maximum and value labels
        # initial value = 50, min value = 0, max value = 100
        self.pslider = wx.Slider(self, -1, 0, -1000, 1000, (10, 10), (300, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
        self.islider= wx.Slider(self, -1, 0, -1000, 1000, (10, 100), (300, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
        self.dslider = wx.Slider(self, -1, 0, -1000, 1000, (10, 200), (300, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS) 
        # respond to changes in slider position ...
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)

    def sliderUpdate(self, event):
        global KP, KD, KI, pid

        #pidUpdate(self.pslider.GetValue())
        print (self.pslider.GetValue())/10000.0
        KP= self.pslider.GetValue()/10000.0
        KI= self.islider.GetValue()/10000.0
        KD= self.dslider.GetValue()/10000.0
        #print "P:", KP, "I:", KI, "D:,", KD,  "sp:", PIXEL_SETPOINT

        #KP = 0.6*KU
        #KI = 2 * KP / PU
        #KD = KP * PU / 8 
        pid.setPID(KP, KI, KD) 



if __name__ == "__main__":
    main()
