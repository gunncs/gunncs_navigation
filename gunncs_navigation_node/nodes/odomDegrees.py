#!/usr/bin/env python
import roslib
roslib.load_manifest('gunncs_navigation_node')
roslib.load_manifest('turtlebot_node')
roslib.load_manifest('tf')
import tf
import rospy
import math
from geometry_msgs.msg import *
from turtlebot_node.msg import *
from nav_msgs.msg import *

'''
Publishes an odom frame im meters and degrees
'''
def main():
    global pub
    rospy.loginfo("converting quaternions")
    rospy.init_node('odomDegrees')
    rospy.Subscriber("/odom", Odometry, odomChange)
    pub = rospy.Publisher("/odom2D", Pose2D)
    rospy.spin()

def odomChange(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([0,0,data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    pub.publish(Pose2D(x, y, yaw*180/math.pi))



main()
