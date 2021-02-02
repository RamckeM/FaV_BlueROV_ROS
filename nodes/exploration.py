#!/usr/bin/env python
from rospy_tutorials.msg import Floats
from fav_control.msg import StateVector2D, StateVector3D
from scipy.stats import entropy
import numpy as np
import rospy
from math import pi, atan2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

TANK_Y = 4.
TANK_X = 2.
TANK_NUMBER_Y = 80
TANK_NUMBER_X = 40
DY = TANK_Y / TANK_NUMBER_Y
DX = TANK_X / TANK_NUMBER_X

X_TOL = 0.01
Y_TOL = 0.01
YAW_TOL = 0.01


class ExplorationNode():
    def __init__(self):
        rospy.init_node("exploration")
        self.x_setpoint_pub = rospy.Publisher("x_setpoint",
                                              StateVector3D,
                                              queue_size=1)
        self.y_setpoint_pub = rospy.Publisher("y_setpoint",
                                              StateVector3D,
                                              queue_size=1)
        self.yaw_setpoint_pub = rospy.Publisher("yaw_setpoint",
                                                StateVector3D,
                                                queue_size=1)

        self.x_setpoint = np.array([0.7, 0.0, 0.0])
        self.y_setpoint = np.array([2.0, 0.0, 0.0])
        self.yaw_setpoint = np.array([pi / 2, 0.0, 0.0])

        self.x_error = 1.0
        self.y_error = 1.0
        self.yaw_error = 1.0

        self.yaw = 0.0
        self.position = np.array([.0, .0])

        # parameters for field of view
        self.beta = pi / 3
        self.radius_max = 1.

        self.legal_moves = [
            np.array([.2, .2]),
            np.array([-.2, .2]),
            np.array([.2, -.2]),
            np.array([-.2, -.2])
        ]
        self.legal_turns = [-pi / 2, pi / 2, pi]
        self.actions = []
        self.total_entropy = 1.0

        self.x_error_sub = rospy.Subscriber("x_control_error",
                                            StateVector2D,
                                            self.x_error_callback,
                                            queue_size=1)
        self.y_error_sub = rospy.Subscriber("y_control_error",
                                            StateVector2D,
                                            self.y_error_callback,
                                            queue_size=1)
        self.yaw_error_sub = rospy.Subscriber("yaw_control_error",
                                              StateVector2D,
                                              self.yaw_error_callback,
                                              queue_size=1)

        # self.nb_cells = TANK_NUMBER_X * TANK_NUMBER_Y
        self.mapping_sub = rospy.Subscriber("mapping", (Floats),
                                            self.mapping_callback,
                                            queue_size=(TANK_NUMBER_X *
                                                        TANK_NUMBER_Y))
        self.map = 0.5 * np.ones((TANK_NUMBER_Y, TANK_NUMBER_X))
        self.entropy = np.zeros((TANK_NUMBER_Y, TANK_NUMBER_X))
        self.position_sub = rospy.Subscriber("ekf_pose",
                                             PoseWithCovarianceStamped,
                                             self.position_callback,
                                             queue_size=1)
        self.orientation_sub = rospy.Subscriber("ground_truth/state",
                                                Odometry,
                                                self.orientation_callback,
                                                queue_size=1)

    def x_error_callback(self, msg):
        self.x_error = msg.velocity

    def y_error_callback(self, msg):
        self.y_error = msg.velocity

    def yaw_error_callback(self, msg):
        self.yaw_error = msg.velocity

    def mapping_callback(self, msg):
        self.map = np.reshape(msg.data, (TANK_NUMBER_Y, TANK_NUMBER_X))

    def position_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y

    def orientation_callback(self, msg):
        orientation = msg.pose.pose.orientation
        orientation_list = [
            orientation.x, orientation.y, orientation.z, orientation.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def check_if_arrived(self):
        if (self.x_error < X_TOL and self.y_error < Y_TOL
                and self.yaw_error < YAW_TOL):
            print("We have arrived at our destination")
            return True
        else:
            print("Not yet arrived")
            return False

    def calculate_entropy(self):
        for i in range(TANK_NUMBER_Y):
            for j in range(TANK_NUMBER_X):
                self.entropy[i][j] = entropy(
                    [self.map[i][j], 1 - self.map[i][j]])
        self.total_entropy = np.sum(np.sum(self.entropy))

    def determine_actions(self):
        self.actions = []
        x = self.position[0]
        y = self.position[1]
        # for move in self.legal_moves:
        # CHECK FOR COLLISIONS
        for turn in self.legal_turns:
            self.actions.append([x, y, self.yaw + turn])

    def expected_infogain(self, action):
        for i in range(TANK_NUMBER_Y):
            for j in range(TANK_NUMBER_X):
                pos = np.array([i * DY + DY / 2, j * DX + DX / 2])
                distance = np.linalg.norm(self.position - pos)
                angle = atan2(pos[1] - self.position[1],
                              pos[0] - self.position[0])
                infogain = 0.0
                if -self.beta < angle < self.beta:
                    if distance < self.radius_max:
                        infogain += self.entropy[i][j]
        return infogain

    def publish_action(self, action):
        print("New Exploration Action: ")
        print(action)
        print("Current Entropy: ")
        print(self.total_entropy)
        self.x_setpoint[0] = action[0]
        self.y_setpoint[0] = action[1]
        self.yaw_setpoint[0] = action[2]

        x_msg = StateVector3D()
        x_msg.header.stamp = rospy.Time.now()
        x_msg.position = self.x_setpoint[0]
        x_msg.velocity = self.x_setpoint[1]
        x_msg.acceleration = self.x_setpoint[2]

        y_msg = StateVector3D()
        y_msg.header.stamp = rospy.Time.now()
        y_msg.position = self.y_setpoint[0]
        y_msg.velocity = self.y_setpoint[1]
        y_msg.acceleration = self.y_setpoint[2]

        yaw_msg = StateVector3D()
        yaw_msg.header.stamp = rospy.Time.now()
        yaw_msg.position = self.yaw_setpoint[0]
        yaw_msg.velocity = self.yaw_setpoint[1]
        yaw_msg.acceleration = self.yaw_setpoint[2]

        self.x_setpoint_pub.publish(x_msg)
        self.y_setpoint_pub.publish(y_msg)
        self.yaw_setpoint_pub.publish(yaw_msg)

    def run(self):
        rate = rospy.Rate(0.2)
        while self.total_entropy > 0:
            if self.check_if_arrived():
                self.calculate_entropy()
                if self.total_entropy > 0:
                    self.determine_actions()
                    infogains = []
                    for action in self.actions:
                        infogains.append(self.expected_infogain(action))
                    i = np.argmax(infogains)
                    self.publish_action(self.actions[i])
                    print("ACTIONS")
                    print(self.actions)
            rate.sleep()
        print("Exploration Complete")


def main():
    node = ExplorationNode()
    node.run()


if __name__ == "__main__":
    main()
