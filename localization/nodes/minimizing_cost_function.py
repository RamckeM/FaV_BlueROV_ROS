#!/usr/bin/env python
import rospy
import numpy as np
from scipy.optimize import minimize
import random
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement

class MinimizationNode():
    def __init__(self):
        rospy.init_node("minimization_localization")

        self.tag1 = np.array([0.5,3.35,-0.5])
        self.tag2 = np.array([1.1,3.35,-0.5])
        self.tag3 = np.array([0.5,3.35,-0.9])
        self.tag4 = np.array([1.1,3.35,-0.9])

        self.distances = np.array([1,1,1,1])
        self.bounds = [(0,2),(0,3.35),(-1.2,0)]
        self.x0 = np.array([0.8,1.5,-0.7])

        self.measurements = np.zeros((4, 2))

        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback, queue_size=1)
        self.position_pub = rospy.Publisher("raw_position", Point, queue_size=1)    
    
    def range_callback(self, range_array_msg):
        number_anchors = len(range_array_msg.measurements) 
        for index, measurement in enumerate(range_array_msg.measurements):
            anchor_id = measurement.id
            range_measurement = measurement.range        
            self.measurements[index, 0] = range_measurement
            self.measurements[index, 1] = anchor_id    
        
    def costfun(self, x):
        y1 = np.linalg.norm(self.tag1-x)
        y2 = np.linalg.norm(self.tag2-x)
        y3 = np.linalg.norm(self.tag3-x)
        y4 = np.linalg.norm(self.tag4-x)
        y = np.array([y1, y2, y3, y4])
        return np.mean((y-self.distances)**2)    
        
    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            x = minimize(self.costfun, self.x0, args=(self), method="TNC", bounds=self.bounds)
            msg = Point()
            msg.data = x
            self.position_pub.publish(msg)

            rate.sleep()
        

def main():
    node = MinimizationNode()
    node.run()
    
if __name__ == "__main__":
    main()
