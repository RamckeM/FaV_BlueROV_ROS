#!/usr/bin/env python
import rospy
from rospy.topics import Subscriber
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math

SIMULATION = 1

class PositionControlNode():
    def __init__(self):
        rospy.init_node("position_controller")

        self.position_setpoint = Point()
        self.estimated_position = Point()
        self.optimal_position = Point()
        self.mcl_position = Point()
        self.postition_setpoint = Point()
        self.position_setpoint.x = 0.5
        self.position_setpoint.y = 1.0
        self.position_setpoint.z = -0.4
        self.yaw_gs = 0.  #ground state angle in pi!  
        self.yaw_mav = 0.  #mavros 
        self.yaw_error = 0.
        self.check_stabilize = Bool()
        self.check_stabilize.data = False
        self.stabilze_point = Point()
        self.stabilze_point.x = 0.7
        self.stabilze_point.y = 2.0
        self.stabilze_point.z = -0.7
        self.kp_long = rospy.get_param('~longitudinal_gain')
        self.kp_lat = rospy.get_param('~lateral_gain')
        self.kp_yaw = rospy.get_param('~yaw_gain')
        self.x0 = 0.0
        self.x1 = 2.0
        self.y0 = 0.0
        self.y1 = 4.0
        #self.position_setpoint.z = -0.5

        # Listens to manual published topic
        self.set_point_sub = rospy.Subscriber("wanted_point", Point, self.set_point_callback, queue_size=1)
        self.estimated_position_sub = rospy.Subscriber("estimated_position", Point, self.estimated_position_callback, queue_size=1)
        self.optimal_position_sub = rospy.Subscriber("opt_position", Point, self.optimal_position_callback, queue_size=1)
        self.mcl_position_sub = rospy.Subscriber("mcl_position", Point, self.mcl_position_callback, queue_size=1)

        self.ground_orientation_sub = rospy.Subscriber("ground_truth/state", Odometry, self.ground_orientation_callback,queue_size=1)
        #self.mavros_orientation_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.mavros_orientation_callback,queue_size=1)
        self.mavros_orientation_sub = rospy.Subscriber("mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, self.mavros_orientation_callback,queue_size=1)
        self.check_stabilize_sub = rospy.Subscriber("bool_stabilize", Bool, self.bool_stabilize_callback, queue_size=1)

        self.depth_setpoint_pub = rospy.Publisher("depth_setpoint", Float64, queue_size=1)
        self.longitudinal_thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)
        self.yaw_thrust_pub = rospy.Publisher("yaw", Float64, queue_size=1)

    def bool_stabilize_callback(self,msg):
        self.check_stabilize.data = msg.data

    def ground_orientation_callback(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_gs = yaw
        #print("gs:  ", yaw)

    def mavros_orientation_callback(self, msg):
        #orientation_q = msg.pose.orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_mav = yaw
        #print("mav: ", yaw)


    def mcl_position_callback(self, msg):
        self.mcl_position.x = msg.x
        self.mcl_position.y = msg.y
        self.mcl_position.z = msg.z

    def optimal_position_callback(self, msg):
        self.optimal_position.x = msg.x
        self.optimal_position.y = msg.y
        self.optimal_position.z = msg.z

    def set_point_callback(self, msg):
        #print("i am here")
        self.position_setpoint.x = msg.x
        self.position_setpoint.y = msg.y
        self.position_setpoint.z = msg.z

        #msg_depth = Float64
        #msg_depth.data = -0.6 # msg.z
        #self.depth_setpoint_pub.publish(msg_depth)


    def estimated_position_callback(self, msg):
        self.estimated_position.x = msg.x
        self.estimated_position.y = msg.y
        self.estimated_position.z = msg.z

    def yaw_controller(self):
        if SIMULATION == 1:
            yaw_error = math.pi/2. - self.yaw_gs
        else:
            yaw_error = math.pi/2. - self.yaw_mav + 0.1
        msg_yaw = Float64()
        msg_yaw.data =  yaw_error * self.kp_yaw
        self.yaw_thrust_pub.publish(msg_yaw)

    def run(self):
        rate = rospy.Rate(100.0)
        counter = 0
        while not rospy.is_shutdown():
            #self.estimated_position = self.optimal_position
            set_depth = Float64()
            if self.check_stabilize.data == False:
                longitudinal_error = self.position_setpoint.x - self.mcl_position.x
                lateral_error = self.position_setpoint.y - self.mcl_position.y
                #print(longitudinal_error, lateral_error)
                set_depth.data = self.position_setpoint.z
                counter = 0
                # print(yaw_error)
                # print(math.pi/2. - self.yaw_mav)                        
            else:
                if counter == 0:
                    # save actual point to stabilize there
                    self.stabilze_point.x = self.mcl_position.x
                    self.stabilze_point.y = self.mcl_position.y 
                    counter = counter + 1              
                longitudinal_error = 0 #self.stabilze_point.x - self.mcl_position.x
                lateral_error = 0 #self.stabilze_point.y - self.mcl_position.y
                set_depth.data = -0.4

            if (self.mcl_position.x < self.x0 or self.mcl_position.x > self.x1 or self.mcl_position.y < self.y0 or self.mcl_position.y > self.y1 
            or self.position_setpoint.x < self.x0 or self.position_setpoint.x > self.x1 or self.position_setpoint.y < self.y0 or self.position_setpoint.y > self.y1):
                longitudinal_error = 0.0
                lateral_error = 0.0

            self.yaw_controller()   
            self.depth_setpoint_pub.publish(set_depth.data) 
            msg_longitudinal = Float64()
            msg_longitudinal.data = lateral_error * self.kp_long #longitudinal_error * 0.25
            self.longitudinal_thrust_pub.publish(msg_longitudinal)
            msg_lateral = Float64()
            msg_lateral.data = -longitudinal_error * self.kp_lat #lateral_error * 0.25  
            self.lateral_thrust_pub.publish(msg_lateral)
            #print("mcl", self.mcl_position.x)
            rate.sleep()


def main():
    node = PositionControlNode()
    node.run()

if __name__ == "__main__":
    main()
