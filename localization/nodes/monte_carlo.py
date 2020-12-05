#!/usr/bin/env python
import rospy
import numpy as np
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement



class LocalizationNode():
    def __init__(self):
        rospy.init_node("monte_carlo.py")

        self.range = 0

        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback, queue_size=1)

    

    def range_callback(self, msg):
        self.range = msg.data
        

    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            pass

        rate.sleep()


def main():
    node = LocalizationNode()
    node.run()

if __name__ == "__main__":
    main()
