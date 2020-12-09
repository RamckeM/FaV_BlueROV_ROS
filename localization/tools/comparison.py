#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class ComparisonNode():
    def __init__(self):
        rospy.init_node("comparison_node")
        self.error_opt_pub = rospy.Publisher("error_opt", Float64, queue_size=1)
        self.error_mcl_pub = rospy.Publisher("error_mcl", Float64, queue_size=1)
        # comparison_pub
        self.ground_truth_sub = rospy.Subscriber("ground_truth/state",
                                                 Odometry,
                                                 self.ground_truth_callback,
                                                 queue_size=1)
        self.opt_sub = rospy.Subscriber("opt_position",
                                        Point,
                                        self.opt_position_callback,
                                        queue_size=1)
        self.mcl_sub = rospy.Subscriber("mcl_position",
                                        Point,
                                        self.mcl_position_callback,
                                        queue_size=1)
        self.opt_position = np.array([0.0, 0.0, 0.0])
        self.mcl_position = np.array([0.0, 0.0, 0.0])
        self.true_position = np.array([0.0, 0.0, 0.0])

    def opt_position_callback(self, opt_position_msg):
        self.opt_position[0] = opt_position_msg.x
        self.opt_position[1] = opt_position_msg.y
        self.opt_position[2] = opt_position_msg.z

    def ground_truth_callback(self, ground_truth_msg):
        self.true_position[0] = ground_truth_msg.pose.pose.position.x
        self.true_position[1] = ground_truth_msg.pose.pose.position.y
        self.true_position[2] = ground_truth_msg.pose.pose.position.z

    def mcl_position_callback(self, mcl_position_msg):
        self.mcl_position[0] = mcl_position_msg.x
        self.mcl_position[1] = mcl_position_msg.y
        self.mcl_position[2] = mcl_position_msg.z

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            error_opt_msg = Float64()
            error_opt_msg.data = np.linalg.norm(self.opt_position -
                                                self.true_position)

            error_mcl_msg = Float64()
            error_mcl_msg.data = np.linalg.norm(self.mcl_position -
                                                self.true_position)

            self.error_opt_pub.publish(error_opt_msg)
            self.error_mcl_pub.publish(error_mcl_msg)

            rate.sleep()


def main():
    node = ComparisonNode()
    node.run()


if __name__ == "__main__":
    main()
