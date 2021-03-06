#!/usr/bin/env python
import rospy
import numpy as np
from scipy import optimize
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from range_sensor.msg import RangeMeasurementArray


TANK_DEPTH = -1
TANK_LENGTH = 4
TANK_WIDTH = 2

SENSOR_POSITION = np.array([0, 0.2, 0.1])

class MinimizationNode():
    def __init__(self):
        rospy.init_node("minimizer")

        # Simulation
        # self.tag_1 = np.array([0.5, 3.35, -0.5])
        # self.tag_2 = np.array([1.1, 3.35, -0.5])
        # self.tag_3 = np.array([0.5, 3.35, -0.9])
        # self.tag_4 = np.array([1.1, 3.35, -0.9])

        # Experiment
        self.tag_1 = np.array([0.5, 3.35, -0.28])
        self.tag_2 = np.array([1.1, 3.35, -0.28])
        self.tag_3 = np.array([0.5, 3.35, -0.68])
        self.tag_4 = np.array([1.1, 3.35, -0.68])

        self.distances = np.array([0.0, 0.0, 0.0, 0.0])

        self.bounds = [(0, TANK_WIDTH), (0, TANK_LENGTH), (TANK_DEPTH, 0)]
        self.x0 = np.array([0.7, 2, -0.7])

        self.position_pub = rospy.Publisher("opt_position", Point, queue_size=1)
        self.error_pub = rospy.Publisher("error_opt_prediction", Float64, queue_size=1)

        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback, queue_size=1)


    def range_callback(self, msg):
        num_measurements = len(msg.measurements)
        if num_measurements:
            for measurement in msg.measurements:
                self.distances[measurement.id - 1] = measurement.range
        else:
            print("WARNING: No measurements received!")


    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            parameters = [self.tag_1, self.tag_2, self.tag_3, self.tag_4, self.distances]
            res = optimize.minimize(costfun, self.x0, args=(parameters), method='TNC', bounds=self.bounds)
            x = res.x
            self.x0 = x
            predicted_error = res.fun
            x -= SENSOR_POSITION

            position_msg = Point()
            position_msg.x = x[0]
            position_msg.y = x[1]
            position_msg.z = x[2]
            self.position_pub.publish(position_msg)
            error_msg = Float64()
            error_msg.data = predicted_error
            self.error_pub.publish(error_msg)

            rate.sleep()


def costfun(x, parameters):
    y0 = np.linalg.norm(parameters[0] - x)
    y1 = np.linalg.norm(parameters[1] - x)
    y2 = np.linalg.norm(parameters[2] - x)
    y3 = np.linalg.norm(parameters[3] - x)
    y = np.array([y0, y1, y2, y3])
    return np.mean((y - parameters[4])**2)
    


def main():
    node = MinimizationNode()
    node.run()

if __name__ == "__main__":
    main()
