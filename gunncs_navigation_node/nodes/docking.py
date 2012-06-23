#!/usr/bin/env python
import roslib
roslib.load_manifest('gunncs_navigation_node')
roslib.load_manifest('turtlebot_node')
import rospy

from geometry_msgs.msg import *
from turtlebot_node.msg import *
from std_msgs.msg import *

def main():
    global pub, pubDocking
    rospy.loginfo("waiting for dock IR")
    rospy.init_node('docking')
    rospy.Subscriber("/turtlebot_node/sensor_state", TurtlebotSensorState, sensStateChange)
    pub = rospy.Publisher("/cmd_vel", Twist)
    pubDocking = rospy.Publisher("/Docking", Bool)
    rospy.spin()

def sensStateChange(data):
    ir = data.remote_opcode
    if (ir == data.REMOTE_NONE):
        return
    else: 
        rospy.loginfo("detected IR forcefield!")
        msg = Twist()
        if (ir >= data.REMOTE_GREEN_BUOY):
               if (ir >= data.REMOTE_RED_BUOY_AND_GREEN_BUOY):
                msg.linear.x = 0.05
                msg.angular.z = 0.0
            elif (ir >= data.REMOTE_RED_BUOY):
                msg.linear.x = 0.05
                msg.angular.z = 0.4
            else:
                msg.linear.x = 0.05
                msg.angular.z = -0.4
            pub.publish(msg)
            pubDocking.publish(True)



main()
