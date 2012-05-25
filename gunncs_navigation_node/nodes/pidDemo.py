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
p_controller = PIDController(KP, 0, 0, -1.0, 1.0)
i_controller = PIDController(0, KI, 0, -1.0, 1.0)
d_controller = PIDController(0, 0, KD, -1.0, 1.0)


def main():
    global cvPub, twistPub, pPub, iPub, dPub, spPub
    rospy.loginfo("starting PIDDemo..")
    rospy.init_node('PIDDemo')

    rospy.Subscriber("/distance", Float32, distanceCb) 
    cvPub = rospy.Publisher('/cv', Float32)
    twistPub = rospy.Publisher('/cmd_vel', Twist)
    pPub= rospy.Publisher('/p', Float32)
    iPub= rospy.Publisher('/i', Float32)
    dPub= rospy.Publisher('/d', Float32)
    spPub= rospy.Publisher('/setpoint', Float32)
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
    global p_controller, i_controller, d_controller, SETPOINT
    global pPub, iPub, dPub, spPub

    #print (data)
    p_controller.update(data, SETPOINT)
    i_controller.update(data, SETPOINT)
    d_controller.update(data, SETPOINT)

    p_result = p_controller.getResult()
    i_result = i_controller.getResult()
    d_result = d_controller.getResult()

    pmsg = Float32()
    pmsg.data = p_result
    pPub.publish(pmsg)

    imsg = Float32()
    imsg.data = i_result
    iPub.publish(imsg)
    
    dmsg = Float32()
    dmsg.data = d_result
    dPub.publish(dmsg)

    spmsg = Float32()
    spmsg.data = SETPOINT
    spPub.publish(spmsg)

    command(p_result + i_result + d_result)



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
        self.pslider = wx.Slider(self, -1, 0, -1000, 1000, (10, 10), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
        self.islider= wx.Slider(self, -1, 0, -1000, 1000, (10, 100), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS)
        self.dslider = wx.Slider(self, -1, 0, -1000, 1000, (10, 200), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS) 
        self.spslider= wx.Slider(self, -1, 750, 0, 2000, (10, 300), (600, 50),
                wx.SL_HORIZONTAL | wx.SL_AUTOTICKS | wx.SL_LABELS) 

        self.plabel = wx.StaticText(self, wx.ID_ANY, label="kp", pos=(10, 10))
        self.ilabel = wx.StaticText(self, wx.ID_ANY, label="ki", pos=(10, 100))
        self.dlabel = wx.StaticText(self, wx.ID_ANY, label="kd", pos=(10, 200))
        self.setpointlabel = wx.StaticText(self, wx.ID_ANY, label="setpoint", pos=(10, 300))
        # respond to changes in slider position ...
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)

    def sliderUpdate(self, event):
        global KP, KD, KI, p_controller, i_controller, d_controller, SETPOINT

        #pidUpdate(self.pslider.GetValue())
        print (self.pslider.GetValue())/10000.0
        KP= self.pslider.GetValue()/10000.0
        KI= self.islider.GetValue()/10000.0
        KD= self.dslider.GetValue()/10000.0
        SETPOINT = self.spslider.GetValue()
        #print "P:", KP, "I:", KI, "D:,", KD,  "sp:", PIXEL_SETPOI

        #KP = 0.6*KU
        #KI = 2 * KP / PU
        #KD = KP * PU / 8 
        p_controller.setPID(KP, 0, 0)
        i_controller.setPID(0, KI, 0)
        d_controller.setPID(0, 0, KD)



if __name__ == "__main__":
    main()
