#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
# from nav_msgs.msg import Odometry


class DataCrunchingNode():
    def __init__(self):
        rospy.init_node("data_crunching_node")
        self.error_opt_mean_pub = rospy.Publisher("error_opt_mean",
                                                  Float64,
                                                  queue_size=1)
        self.error_mcl_mean_pub = rospy.Publisher("error_mcl_mean",
                                                  Float64,
                                                  queue_size=1)
        self.opt_position_mean_pub = rospy.Publisher("opt_position_mean",
                                                     Point,
                                                     queue_size=1)
        self.mcl_position_mean_pub = rospy.Publisher("mcl_position_mean",
                                                     Point,
                                                     queue_size=1)
        self.error_opt_sub = rospy.Subscriber("error_opt",
                                              Float64,
                                              self.error_opt_callback,
                                              queue_size=1)
        self.error_mcl_sub = rospy.Subscriber("error_mcl",
                                              Float64,
                                              self.error_mcl_callback,
                                              queue_size=1)
        self.opt_position_sub = rospy.Subscriber("opt_position",
                                                 Point,
                                                 self.opt_position_callback,
                                                 queue_size=1)
        self.mcl_position_sub = rospy.Subscriber("mcl_position",
                                                 Point,
                                                 self.mcl_position_callback,
                                                 queue_size=1)
        self.opt_position_bulk = []
        self.mcl_position_bulk = []
        self.error_opt_bulk = []
        self.error_mcl_bulk = []

    def error_opt_callback(self, error_opt_msg):
        self.error_opt_bulk.append(error_opt_msg.data)
        if len(self.error_opt_bulk) > 100:
            self.error_opt_bulk.pop(0)

    def error_mcl_callback(self, error_mcl_msg):
        self.error_mcl_bulk.append(error_mcl_msg.data)
        if len(self.error_mcl_bulk) > 100:
            self.error_mcl_bulk.pop(0)

    def opt_position_callback(self, opt_position_msg):
        opt_position = np.array(
            [opt_position_msg.x, opt_position_msg.y, opt_position_msg.z])
        self.opt_position_bulk.append(opt_position)
        if len(self.opt_position_bulk) > 20:
            self.opt_position_bulk.pop(0)

    def mcl_position_callback(self, mcl_position_msg):
        mcl_position = np.array(
            [mcl_position_msg.x, mcl_position_msg.y, mcl_position_msg.z])
        self.mcl_position_bulk.append(mcl_position)
        if len(self.mcl_position_bulk) > 20:
            self.mcl_position_bulk.pop(0)

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            error_opt_mean_msg = Float64()
            error_opt_mean_msg.data = np.mean(self.error_opt_bulk)

            error_mcl_mean_msg = Float64()
            error_mcl_mean_msg.data = np.mean(self.error_mcl_bulk)

            self.error_opt_mean_pub.publish(error_opt_mean_msg)
            self.error_mcl_mean_pub.publish(error_mcl_mean_msg)

            rate.sleep()


def main():
    node = DataCrunchingNode()
    node.run()


if __name__ == "__main__":
    main()
