#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from range_sensor.msg import RangeMeasurementArray
from geometry_msgs.msg import Point
# from nav_msgs.msg import Odometry


class RangeAnalysisNode():
    def __init__(self):
        rospy.init_node("range_analysis_node")
        self.range_sub = rospy.Subscriber("ranges",
                                          RangeMeasurementArray,
                                          self.range_callback,
                                          queue_size=1)
        self.range1_mean_pub = rospy.Publisher("range1_mean",
                                               Float64,
                                               queue_size=1)
        self.range1_variance_pub = rospy.Publisher("range1_deviation",
                                                   Float64,
                                                   queue_size=1)
        self.range2_mean_pub = rospy.Publisher("range2_mean",
                                               Float64,
                                               queue_size=1)
        self.range2_variance_pub = rospy.Publisher("range2_deviation",
                                                   Float64,
                                                   queue_size=1)
        self.range3_mean_pub = rospy.Publisher("range3_mean",
                                               Float64,
                                               queue_size=1)
        self.range3_variance_pub = rospy.Publisher("range3_deviation",
                                                   Float64,
                                                   queue_size=1)
        self.range4_mean_pub = rospy.Publisher("range4_mean",
                                               Float64,
                                               queue_size=1)
        self.range4_variance_pub = rospy.Publisher("range4_deviation",
                                                   Float64,
                                                   queue_size=1)

        self.ranges_bulk = [[], [], [], []]

    def range_callback(self, msg):
        num_measurements = len(msg.measurements)
        if num_measurements:
            for measurement in msg.measurements:
                self.ranges_bulk[measurement.id - 1].append(measurement.range)
                if len(self.ranges_bulk[measurement.id - 1]) > 200:
                    self.ranges_bulk[measurement.id - 1].pop(0)
        else:
            print("WARNING: No measurements received!")

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            range1_mean_msg = Float64()
            range1_variance_msg = Float64()
            range2_mean_msg = Float64()
            range2_variance_msg = Float64()
            range3_mean_msg = Float64()
            range3_variance_msg = Float64()
            range4_mean_msg = Float64()
            range4_variance_msg = Float64()

            range1_mean_msg.data = np.mean(self.ranges_bulk[0])
            range1_variance_msg.data = np.sqrt(np.var(self.ranges_bulk[0]))
            range2_mean_msg.data = np.mean(self.ranges_bulk[1])
            range2_variance_msg.data = np.sqrt(np.var(self.ranges_bulk[1]))
            range3_mean_msg.data = np.mean(self.ranges_bulk[2])
            range3_variance_msg.data = np.sqrt(np.var(self.ranges_bulk[2]))
            range4_mean_msg.data = np.mean(self.ranges_bulk[3])
            range4_variance_msg.data = np.sqrt(np.var(self.ranges_bulk[3]))

            self.range1_mean_pub.publish(range1_mean_msg)
            self.range1_variance_pub.publish(range1_variance_msg)
            self.range2_mean_pub.publish(range2_mean_msg)
            self.range2_variance_pub.publish(range2_variance_msg)
            self.range3_mean_pub.publish(range3_mean_msg)
            self.range3_variance_pub.publish(range3_variance_msg)
            self.range4_mean_pub.publish(range4_mean_msg)
            self.range4_variance_pub.publish(range4_variance_msg)

            rate.sleep()


def main():
    node = RangeAnalysisNode()
    node.run()


if __name__ == "__main__":
    main()
