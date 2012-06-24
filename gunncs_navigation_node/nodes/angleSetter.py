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
AngleSetter.py commands the base to maintain centered in the lane

'''

TUNING = True

SETPOINT = 0
LINEAR_SPEED = 20/1000.0

KP = 99.0/1000.0
KI = 0 
KD = 0 

pid = PIDController(KP, KI, KD, -1.0, 1.0)

def main():
    global cvPub, twistPub, pPub, iPub, dPub, spPub
    rospy.loginfo("starting AngleSetter..")
    rospy.init_node('AngleSetter')

    rospy.Subscriber("/gunncs/angle", Float32, angleCb) 
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

def angleCb(distance):
    '''
    ros callback for wall distance, measured by WallSensor
    '''
    pidUpdate(distance.data)

def pidUpdate(data):
    global pid, SETPOINT

    #print (data)
    pid.update(data, SETPOINT)
    result = pid.getResult()
    command(result)



def command(pidresult):
    '''
    commands wall angle with the pid result
    '''
    global cvPub, twistPub, LINEAR_SPEED
    cv_msg = Float32()
    cv_msg.data = pidresult
    cvPub.publish(cv_msg)

    twist_msg = Twist()
    twist_msg.angular.z = pidresult
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

        
        '''
        xSlider::wxSlider   (   wxWindow *  parent,
                wxWindowID  id,
                int     value,
                int     minValue,
                int     maxValue,
                const wxPoint &     pos = wxDefaultPosition,
                const wxSize &  size = wxDefaultSize,
                long    style = wxSL_HORIZONTAL,
                const wxValidator &     validator = wxDefaultValidator,
                const wxString &    name = wxSliderNameStr 
                )       
        '''


        # wx.SL_VERTICAL  displays the slider vertically
        # wx.SL_HORIZONTAL  displays the slider horizontally
        # wx.SL_AUTOTICKS  displays tick marks
        # wx.SL_LABELS  displays minimum, maximum and value labels
        # initial value = 50, min value = 0, max value = 100
        self.pslider = wx.Slider(self, -1, 99, -1000, 1000, (10, 10), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
        self.islider= wx.Slider(self, -1, 0, -1000, 1000, (10, 100), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
        self.dslider = wx.Slider(self, -1, 0, -1000, 1000, (10, 200), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS) 
        self.sslider = wx.Slider(self, -1, 20, -1000, 1000, (10, 300), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS) 

        self.plabel = wx.StaticText(self, wx.ID_ANY, label="kp", pos=(10, 10))
        self.ilabel = wx.StaticText(self, wx.ID_ANY, label="ki", pos=(10, 100))
        self.dlabel = wx.StaticText(self, wx.ID_ANY, label="kd", pos=(10, 200))
        self.slabel = wx.StaticText(self, wx.ID_ANY, label="s", pos=(10, 300))
        self.setpointlabel = wx.StaticText(self, wx.ID_ANY, label="setpoint", pos=(10, 300))
        # respond to changes in slider position ...
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)

    def sliderUpdate(self, event):
        global KP, KD, KI, pid, LINEAR_SPEED

        #pidUpdate(self.pslider.GetValue())
        #print (self.pslider.GetValue())/10000.0
        KP= self.pslider.GetValue()/1000.0
        KI= self.islider.GetValue()/1000.0
        KD= self.dslider.GetValue()/1000.0
        LINEAR_SPEED = self.sslider.GetValue()/1000.0;
        #print "P:", KP, "I:", KI, "D:,", KD,  "sp:", PIXEL_SETPOI

        #KP = 0.6*KU
        #KI = 2 * KP / PU
        #KD = KP * PU / 8 
        pid.setPID(KP, KI, KD)


if __name__ == "__main__":
    main()
