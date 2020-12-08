#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point


class PositionControlNode():
    def __init__(self):
        rospy.init_node("position_controller")

        self.position_setpoint = Point()
        self.estimated_position = Point()

        # Listens to manual published topic
        self.position_setpoint_sub = rospy.Subscriber("position_setpoint", Point, self.position_setpoint_callback, queue_size=1)
        self.estimated_position_sub = rospy.Subscriber("estimated_position", Point, self.estimated_position_callback, queue_size=1)

        self.depth_setpoint_pub = rospy.Publisher("depth_setpoint", Float64, queue_size=1)
        self.longitudinal_thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)


    def position_setpoint_callback(self, msg):
        self.position_setpoint.x = msg.x
        self.position_setpoint.y = msg.y
        self.position_setpoint.z = msg.z

        msg_depth = Float64
        msg_depth.data =  msg.z
        self.depth_setpoint_pub.publish(msg_depth)


    def estimated_position_callback(Self, msg):
        self.estimated_position.x = msg.x
        self.estimated_position.y = msg.y
        self.estimated_position.z = msg.z


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            longitudinal_error = self.position_setpoint.x - self.estimated_position.x
            lateral_error = self.position_setpoint.y - self.estimated_position.y

            msg_longitudinal = Float64()
            msg_longitudinal.data = longitudinal_error
            self.longitudinal_thrust_pub.publish(msg_longitudinal)
            msg_lateral = Float64()
            msg_lateral.data = lateral_error
            self.lateral_thrust_pub.publish(msg_lateral)

            rate.sleep()


def main():
    node = PositionControlNode()
    node.run()

if __name__ == "__main__":
    main()
