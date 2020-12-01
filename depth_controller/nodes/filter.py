#!/usr/bin/env python
import rospy
import numpy as np
from scipy.signal import butter, filtfilt
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import FluidPressure


def butter_lowpass(order, bandwidth, fs, data):
    wn = 2 * bandwidth / fs
    b, a = butter(order, wn, btype='low', analog=False)
    return filtfilt(b, a, data)


def pressure_callback(pressure_msg, publisher):
    #filtered_pressure_msg = FluidPressure()
    filtered_pressure_msg = Float32()
    filtered_pressure_msg.data = butter_lowpass(2, 5, 100, pressure_msg)
    publisher.publish(filtered_pressure_msg)


def main():
    rospy.init_node("filter")

    filtered_pub = rospy.Publisher("filtered_data", Float32, queue_size=1)
    #pressure_sub = rospy.Subscriber("pressure", FluidPressure, pressure_callback, filtered_pub)
    pressure_sub = rospy.Subscriber("depth", Float32, pressure_callback, filtered_pub)

    rospy.spin()

if __name__ == "__main__":
    main()
