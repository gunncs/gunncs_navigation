#!/usr/bin/env python
import roslib; roslib.load_manifest('gunncs_navigation_node')

import rospy
import serial

#from std_msgs.msg import *
from sensor_msgs.msg import Range

'''
takes data from an arduino and publishes to Ranges



'''

NUM_SENSORS = 6
DEVICE_ADDRESS = "/dev/ttyUSB0"
BAUD_RATE = 9600

def main():
    global NUM_SENSORS, BAUD_RATE, DEVICE_ADDRESS
    rospy.loginfo("Launching arduino analog reader")
    rospy.init_node("arduinoReader")

    ser = serial.Serial(DEVICE_ADDRESS)
    ser.baudrate = BAUD_RATE

    pubs = []
    for index in range(NUM_SENSORS):
        pubs.append(rospy.Publisher(("/sonar/" + str(index)), Range))

    while not rospy.is_shutdown():
        data = ser.readline().split("\t")
        data.pop() # last element is garbage
        rospy.loginfo("Read line: " + str(data))
        for reading in data:
            #construct message
            msg = Range()
            try:
                msg.range = float(reading)
            except ValueError:
                msg.range = -1.0

            pubIndex = data.index(reading)
            if pubIndex < NUM_SENSORS :
                pubs[pubIndex].publish(msg)


if __name__ == "__main__":
    main()

