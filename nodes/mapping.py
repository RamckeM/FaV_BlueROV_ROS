#!/usr/bin/env python
# from operator import gt
# from numpy.lib.function_base import gradient
# from numpy.ma.core import left_shift, reshape
import rospy
import time
# from rospy.topics import Subscriber
# from std_msgs.msg import Float64, Bool
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

TANK_Z = -1.
TANK_Y = 4.
TANK_X = 2.

TANK_NUMBER_Y = 80  #rospy.get_param('~TANK_NUMBER_Y')
TANK_NUMBER_X = 40  #rospy.get_param('~TANK_NUMBER_X')


class MappingNode():
    def __init__(self):
        rospy.init_node("mapping")
        # --- algorithm variables
        self.tank_number_y = int(rospy.get_param('~tank_number_y'))
        self.tank_number_x = int(rospy.get_param('~tank_number_x'))
        self.l_occ = 5.0e-1
        self.l_free = -5.0e-1
        self.l_0 = 0.0
        self.map = np.zeros((self.tank_number_y, self.tank_number_x))

        self.cell_dx = TANK_X / self.tank_number_x
        self.cell_dy = TANK_Y / self.tank_number_y
        self.position = np.array([0.7, 2.0])
        self.yaw = math.pi / 2  # ground state angle in pi!

        self.beta = math.pi / 3  # angle of view
        self.radius_max = 1.

        self.obstacles = []
        self.obstacle_shadows = []
        # publishing
        self.mapping_pub = rospy.Publisher("mapping", (Floats),
                                           queue_size=(self.tank_number_y *
                                                       self.tank_number_x))
        self.mapping_param_pub = rospy.Publisher("mapping_param", (Floats),
                                                 queue_size=(2))
        # subscribing
        self.obstacle_sub = rospy.Subscriber("objects",
                                             ObjectDetectionArray,
                                             self.obstacle_callback,
                                             queue_size=1)
        self.position_sub = rospy.Subscriber("ekf_pose",
                                             PoseWithCovarianceStamped,
                                             self.position_callback,
                                             queue_size=1)
        self.ground_orientation_sub = rospy.Subscriber(
            "ground_truth/state",
            Odometry,
            self.ground_orientation_callback,
            queue_size=1)

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

    def obstacle_callback(self, msg):
        for obstacle in msg.detections:
            self.obstacles.append([
                obstacle.pose.position.x, obstacle.pose.position.y,
                obstacle.size
            ])

    def map_updating(self):
        self.obstacle_shadows = []
        for obstacle in self.obstacles:
            obst_pos = np.array([obstacle[0], obstacle[1]])
            obst_dist = np.linalg.norm(self.position - obst_pos)
            obst_angle = math.atan2(obst_pos[1] - self.position[1],
                                    obst_pos[0] - self.position[0])
            dphi = math.atan2(obstacle[2] / 2, obst_dist)
            self.obstacle_shadows.append([obst_dist, obst_angle, dphi])

        for i in range(self.tank_number_y):
            for j in range(self.tank_number_x):
                pos = np.array([
                    j * self.cell_dx + self.cell_dx / 2,
                    i * self.cell_dy + self.cell_dy / 2
                ])
                if self.is_in_vision(pos):
                    # check if object
                    l_update = self.l_free
                    for obstacle in self.obstacles:
                        obst_pos = np.array([obstacle[0], obstacle[1]])
                        if np.linalg.norm(
                                obst_pos -
                                pos) < obstacle[2] / 2. + self.cell_dx / 2.:
                            l_update = self.l_occ
                    self.map[i][j] += l_update

    def is_in_vision(self, pos):
        dist = np.linalg.norm(self.position - pos)
        angle = math.atan2(pos[1] - self.position[1], pos[0] - self.position[0])
        leftedge = -self.beta / 2 + self.yaw
        rightedge = self.beta / 2 + self.yaw

        if leftedge < -math.pi:
            lower1 = leftedge + (2 * math.pi) #% (2 * math.pi) #- math.pi
            upper1 = math.pi
            lower2 = -math.pi
            upper2 = rightedge
        elif rightedge > math.pi:
            lower1 = leftedge
            upper1 = math.pi
            lower2 = -math.pi
            upper2 = rightedge - (2 * math.pi) #- math.pi
        else:
            lower1 = leftedge
            upper1 = rightedge
            lower2 = leftedge
            upper2 = rightedge

        if (lower1 < angle < upper1 or lower2 < angle < upper2):
            if dist < self.radius_max:
                isinshadow = False
                for shadow in self.obstacle_shadows:
                    if dist > shadow[0]:
                        if shadow[1] - shadow[2] < angle < shadow[1] + shadow[2]:
                            isinshadow = True
                if not isinshadow:
                    return True
        return False

    def run(self):
        time.sleep(10)
        print("--- start mapping ---")
        while not rospy.is_shutdown():
            rate = rospy.Rate(1.0)
            self.map_updating()
            map_vector = np.reshape(
                self.map, (self.tank_number_x * self.tank_number_y, 1))
            map_publish = 1.0 - 1.0 / (1.0 + np.exp(map_vector))
            # map = np.reshape(map,(self.tank_number_y,self.tank_number_x))
            self.mapping_pub.publish(map_publish)
            self.mapping_param_pub.publish(
                np.array(
                    [self.tank_number_x, self.tank_number_y, TANK_X, TANK_Y]))
            rate.sleep()


def main():
    node = MappingNode()
    node.run()


if __name__ == "__main__":
    main()
