#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from operator import gt
from numpy.lib.function_base import gradient
from numpy.ma.core import left_shift, reshape
import rospy
import time
from rospy.topics import Subscriber
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from fav_object_detection.msg import ObjectDetectionArray, ObjectDetection
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, \
    TwistWithCovarianceStamped,Point
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion
import pylab
from pathlib import Path
from numpy import savetxt
from numpy import asarray
from fav_control.msg import StateVector2D, StateVector3D
import csv   


TANK_Z = -1.
TANK_Y = 4.
TANK_X = 2.

TANK_NUMBER_Y = 80
TANK_NUMBER_X = 40


class PlottingNode():
    def __init__(self):
        rospy.init_node("mapping_plot")
        self.tank_number_y = int(rospy.get_param('~tank_number_y'))
        self.tank_number_x = int(rospy.get_param('~tank_number_x'))
        self.nb_cells = self.tank_number_x*self.tank_number_y
        self.cell_dx = TANK_X/self.tank_number_x
        self.cell_dy = TANK_Y/self.tank_number_y
        self.mapping_sub = rospy.Subscriber("mapping",(Floats),self.mapping_callback,queue_size=(self.nb_cells))
        self.map = 0.5*np.ones(self.nb_cells)
        self.position = np.array([0.7, 2.0])
        self.yaw = math.pi/2.
        self.x_setpoint = 0.7
        self.y_setpoint = 2.0
        self.yaw_setpoint = -1.57

        self.position_sub = rospy.Subscriber("ekf_pose",
                                             PoseWithCovarianceStamped,
                                             self.position_callback,
                                             queue_size=1)
        
        self.ground_orientation_sub = rospy.Subscriber(
            "ground_truth/state",
            Odometry,
            self.ground_orientation_callback,
            queue_size=1)

        self.x_setpoint_sub = rospy.Subscriber("x_setpoint",StateVector3D,self.x_setpoint_callback,queue_size=1)
        self.y_setpoint_sub = rospy.Subscriber("y_setpoint",StateVector3D,self.y_setpoint_callback,queue_size=1)
        self.yaw_setpoint_sub = rospy.Subscriber("yaw_setpoint",StateVector3D,self.yaw_setpoint_callback,queue_size=1)

    def x_setpoint_callback(self,msg):
        self.x_setpoint = msg.position

    def y_setpoint_callback(self,msg):
        self.y_setpoint = msg.position

    def yaw_setpoint_callback(self,msg):
        self.yaw_setpoint = msg.position

    def mapping_callback(self,msg):
        self.map = msg.data

    def ground_orientation_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def position_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y

    def run(self):
        time.sleep(9)
        rate = rospy.Rate(10.0)
        print("--- start plottin ---")
        while not rospy.is_shutdown():
            with open(r'/home/hendrik/fav/catkin_ws/src/mapping_package/data.csv', 'ab') as f:
                writer = csv.writer(f)
                safe = np.reshape(self.map,(self.nb_cells,1))
                # use this for naviagtion instead of 105
                #safe = np.concatenate((np.array([self.position[0],self.position[1],self.yaw, self.x_setpoint,self.y_setpoint,self.yaw_setpoint])), axis=None)
                safe = np.concatenate((np.array([self.position[0],self.position[1],self.yaw, self.x_setpoint,self.y_setpoint,self.yaw_setpoint]), safe), axis=None)
                writer.writerow(safe)
            rate.sleep()
           


def main():
    node = PlottingNode()
    node.run()

if __name__ == "__main__":
    main()
