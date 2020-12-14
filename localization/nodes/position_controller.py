#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


TANK_DEPTH = -1
TANK_LENGTH = 4
TANK_WIDTH = 2

class PositionControlNode():
    def __init__(self):
        rospy.init_node("position_controller")

        self.mcl_position = Point()
        self.opt_position = Point()
        self.position_setpoint = Point()
    
        self.position_setpoint.x = 0.7
        self.position_setpoint.y = 2.0
        self.position_setpoint.z = -0.7

        self.yaw_ground = 0.0
        self.yaw_mavros = 0.0

        self.is_simulation = rospy.get_param('~is_simulation', 'True')
        self.yaw_gain = rospy.get_param('~yaw_gain')
        self.longitudinal_gain = rospy.get_param('~longitudinal_gain')
        self.lateral_gain = rospy.get_param('~lateral_gain')

        # Listens to manual published topic
        self.setpoint_sub = rospy.Subscriber("position_setpoint", Point, self.setpoint_callback, queue_size=1)

        self.mcl_position_sub = rospy.Subscriber("mcl_position", Point, self.mcl_position_callback, queue_size=1)
        self.opt_position_sub = rospy.Subscriber("opt_position", Point, self.opt_position_callback, queue_size=1)
        
        self.ground_orientation_sub = rospy.Subscriber("ground_truth/state", Odometry, self.ground_orientation_callback,queue_size=1)
        self.mavros_orientation_sub = rospy.Subscriber("mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, self.mavros_orientation_callback,queue_size=1)

        self.depth_setpoint_pub = rospy.Publisher("depth_setpoint", Float64, queue_size=1)
        self.yaw_thrust_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        self.longitudinal_thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)


    def ground_orientation_callback(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_ground = yaw

    def mavros_orientation_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_mavros = yaw


    def mcl_position_callback(self, msg):
        self.mcl_position.x = msg.x
        self.mcl_position.y = msg.y
        self.mcl_position.z = msg.z

    def opt_position_callback(self, msg):
        self.opt_position.x = msg.x
        self.opt_position.y = msg.y
        self.opt_position.z = msg.z

    def setpoint_callback(self, msg):
        self.position_setpoint.x = msg.x
        self.position_setpoint.y = msg.y
        self.position_setpoint.z = msg.z


    def yaw_controller(self):
        if self.is_simulation is True:
            yaw_error = math.pi/2.0 - self.yaw_ground
        else:
            yaw_error = math.pi/2.0 - self.yaw_mavros

        msg_yaw = Float64()
        msg_yaw.data =  yaw_error * self.yaw_gain
        self.yaw_thrust_pub.publish(msg_yaw)


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            lateral_error = self.position_setpoint.x - self.mcl_position.x
            longitudinal_error = self.position_setpoint.y - self.mcl_position.y

            if (self.mcl_position.x < 0 or self.mcl_position.x > TANK_WIDTH or self.mcl_position.y < 0 or self.mcl_position.y > TANK_LENGTH
            or self.position_setpoint.x < 0 or self.position_setpoint.x > TANK_WIDTH or self.position_setpoint.y < 0 or self.position_setpoint.y > TANK_LENGTH):
                lateral_error = 0.0
                longitudinal_error = 0.0
            else:
                lateral_output = lateral_error * self.lateral_gain
                longitudinal_output = longitudinal_error * self.longitudinal_gain
                
            self.yaw_controller()   

            depth_setpoint = Float64()
            depth_setpoint.data = self.position_setpoint.z
            self.depth_setpoint_pub.publish(depth_setpoint)

            msg_longitudinal = Float64()
            msg_longitudinal.data = longitudinal_output
            self.longitudinal_thrust_pub.publish(msg_longitudinal)

            msg_lateral = Float64()
            msg_lateral.data = lateral_output
            self.lateral_thrust_pub.publish(msg_lateral)

            rate.sleep()


def main():
    node = PositionControlNode()
    node.run()

if __name__ == "__main__":
    main()
