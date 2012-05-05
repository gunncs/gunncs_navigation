#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')
import rospy
import tf

from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

import serial
import time
import math

#from sensor_driver import *

ORIENTATION_COVARIANCE = [ 1e-6, 0, 0,
                            0, 1e-6, 0,
                            0, 0, 1e-6]

ANGULAR_VELOCITY_COVARIANCE= [ 1e-6, 0, 0,
                            0, 1e-6, 0,
                            0, 0, 1e-6]

ACCELERATION_COVARIANCE= [ 1e-6, 0, 0,
                            0, 1e-6, 0,
                            0, 0, 1e-6]


def main():
    rospy.loginfo("Launching IMU Reader")
    rospy.init_node('IMUReader')
    pub = rospy.Publisher('imu_data', Imu)

    '''
    pubwx = rospy.Publisher('/robotPose/wx', Float64)
    pubwy = rospy.Publisher('/robotPose/wy', Float64)
    pubwz = rospy.Publisher('/robotPose/wz', Float64)
    
    pubax = rospy.Publisher('/robotPose/ax', Float64)
    pubay = rospy.Publisher('/robotPose/ay', Float64)
    pubaz = rospy.Publisher('/robotPose/az', Float64)
    '''

    bcaster = tf.TransformBroadcaster()

    ser = serial.Serial("/dev/ttyACM0")	
    ser.baudrate = 921600

    GRAVITY_ACCEL = 9.80665
    accelX = 0
    accelY = 0
    accelZ = 0
    gyroX = 0
    gyroY = 0
    gyroZ = 0
    quatW = 1
    quatX = 0
    quatY = 0
    quatZ = 0

    rospy.loginfo("beginning read...")
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # update data
        line = ser.readline()
        imu_data = decodeUSBRPCMessage(line)
        if imu_data != False:
            key,data = imu_data
            if key == 94:
                accelX = data/GRAVITY_ACCEL
            elif key == 95:
                accelY = data/GRAVITY_ACCEL
            elif key == 96:
                accelZ = data/GRAVITY_ACCEL 
            elif key == 97:
                gyroX = math.radians(data)
            elif key == 98:
                gyroY = math.radians(data)
            elif key == 99:
                gyroZ = math.radians(data)
            elif key == 100:
                quatW = data  
            elif key == 101:
                quatX = data 
            elif key == 102:
                quatY = data 
            elif key == 103:
                quatZ = data	
            try:
                #dataList = accelX, accelY, accelZ, gyroX, gyroY, gyroZ, quatW, quatX, quatY, quatZ

                dataList = quatW, quatX, quatY, quatZ
                rospy.loginfo(dataList)
                #print(dataList)
            except UnboundLocalError: pass

            #print(key, data)

        # package and send data
        imu_msg = Imu(header=rospy.Header(frame_id="imu_link"))

        imu_quat = (quatX, quatY, quatZ, quatW)
        imu_omegas = (gyroX, gyroY, gyroZ)
        imu_accels = (accelX, accelY, accelZ)

        imu_msg.header.stamp = current_time
        imu_msg.orientation = Quaternion(*imu_quat)
        imu_msg.angular_velocity = Vector3(*imu_omegas)
        imu_msg.linear_acceleration = Vector3(*imu_accels)

        imu_msg.orientation_covariance = ORIENTATION_COVARIANCE;
        imu_msg.angular_velocity_covariance = ANGULAR_VELOCITY_COVARIANCE;
        imu_msg.linear_acceleration_covariance = ACCELERATION_COVARIANCE
        pub.publish(imu_msg)

        '''
        pubwx.publish(Float64(gyroX))
        pubwy.publish(Float64(gyroY))
        pubwz.publish(Float64(gyroZ))

        pubax.publish(Float64(accelX))
        pubay.publish(Float64(accelY))
        pubaz.publish(Float64(accelZ))
        '''

        #transform = (0, 0, 0), imu_quat
        bcaster.sendTransform((0,0,0), imu_quat, rospy.Time.now(), "imu", "world")




def decodeUSBRPCMessage(message):
    if(message[0:3] != 'USB'):
        return False
    colonLocation = message.find(':')
    messageKey = message[3:colonLocation]
    messageData = message[colonLocation+1:]
    try:
        key = int(messageKey)
        data = float(messageData)
    except ValueError:
        return False
    return key, data

main()

