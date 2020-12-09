#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion


SIMULATION = True

class PositionControlNode():
    def __init__(self):
        rospy.init_node("position_controller")

        self.position_setpoint = Point()
        self.estimated_position = Point()

        self.yaw_ground_truth = 0.0
        self.yaw_mavros = 0.0

        # Listens to manual published topic
        self.position_setpoint_sub = rospy.Subscriber("position_setpoint", Point, self.position_setpoint_callback, queue_size=1)
        self.estimated_position_sub = rospy.Subscriber("estimated_position", Point, self.estimated_position_callback, queue_size=1)
        
        self.ground_truth_sub = rospy.Subscriber("ground_truth/state", Odometry, self.ground_truth_callback, queue_size=1)
        self.mavros_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.mavros_pose_callback, queue_size=1)

        self.depth_setpoint_pub = rospy.Publisher("depth_setpoint", Float64, queue_size=1)
        self.yaw_thrust_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.longitudinal_thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)


    def ground_truth_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_ground_truth = yaw

    
    def mavros_pose_callback(self, msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_mavros = yaw


    def position_setpoint_callback(self, msg):
        self.position_setpoint.x = msg.x
        self.position_setpoint.y = msg.y
        self.position_setpoint.z = msg.z

        msg_depth = Float64
        msg_depth.data =  msg.z
        self.depth_setpoint_pub.publish(msg_depth)


    def estimated_position_callback(self, msg):
        self.estimated_position.x = msg.x
        self.estimated_position.y = msg.y
        self.estimated_position.z = msg.z


    def yaw_controller(self):
        if SIMULATION is True:
            yaw_error = math.pi/2. - self.yaw_ground_truth
        else:
            yaw_error = math.pi/2. - self.yaw_mavros
        
        msg = Float64()
        msg.data = yaw_error * 0.25
        self.yaw_thrust


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            self.yaw_controller()
            
            longitudinal_error = self.position_setpoint.x - self.estimated_position.x
            lateral_error = self.position_setpoint.y - self.estimated_position.y

            msg_longitudinal = Float64()
            msg_longitudinal.data = longitudinal_error * 0.5
            self.longitudinal_thrust_pub.publish(msg_longitudinal)
            msg_lateral = Float64()
            msg_lateral.data = lateral_error * 0.5
            self.lateral_thrust_pub.publish(msg_lateral)

            rate.sleep()


def main():
    node = PositionControlNode()
    node.run()

if __name__ == "__main__":
    main()
