#!/usr/bin/env python
import rospy
import numpy as np
import random
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement


TANK_HEIGHT = 1
TANK_LENGTH = 3.8
TANK_WIDTH = 2.7

PARTICLE_COUNT = 200


class LocalizationNode():
    def __init__(self):
        rospy.init_node("localization")

        self.tag_1 = np.array([0.5,3.35,-0.5])
        self.tag_2 = np.array([1.1,3.35,-0.5])
        self.tag_3 = np.array([0.5,3.35,-0.9])
        self.tag_4 = np.array([1.1,3.35,-0.9])

        self.num_particles = PARTICLE_COUNT
        self.particles = np.array([[0,0,0]])
# make array
        self.weights = []

        self.ranges = np.array([0.0,0.0,0.0,0.0])

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.initialize_particles()

        self.position_pub = rospy.Publisher("estimated_position", Point, queue_size=1)
        #self.yaw_pub = rospy.Publisher("yaw", Float64, queue_size=1)
        #self.particle_count_pub = rospy.Publisher("particle_count", Float64, queue_size=1)
        
        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback, queue_size=1)
        self.orientation_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.orientation_callback, queue_size=1)
    

    def range_callback(self, msg):
        for measurement in msg.measurements:
            self.ranges[measurement.id-1] = measurement.range


    def orientation_callback(self, msg):
        pass
        # x = msg.orientation.x
        # y = msg.orientation.y
        # z = msg.orientation.z
        # w = msg.orientation.w
        # self.roll = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
        # self.pitch = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
        # self.yaw = math.asin(2*x*y + 2*z*w)        


    def euclidean_dist(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) **2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2)


    def initialize_particles(self):
        for i in range(self.num_particles):
            x = round(random.random(), 4) * TANK_LENGTH
            y = round(random.random(), 4) * TANK_WIDTH
            z = round(random.random(), 4) * TANK_HEIGHT
            self.particles = np.append(self.particles, [[x, y, z]], axis=0)

            # self.particles[n] returns n'th particle
            # self.particles[n][m] returns m'th coordinate of nth particle


    def dead_reckon(self):
        pass
# Use IMU Data
# Add Gaussian Noise


    def weighting(self):
        weights = []
        for i in range(self.num_particles):
            diff_1 = self.euclidean_dist(self.particles[i], self.tag_1) - self.ranges[0]
            diff_2 = self.euclidean_dist(self.particles[i], self.tag_2) - self.ranges[1]
            diff_3 = self.euclidean_dist(self.particles[i], self.tag_3) - self.ranges[2]
            diff_4 = self.euclidean_dist(self.particles[i], self.tag_4) - self.ranges[3]

            mean_diff = (diff_1 + diff_2 + diff_3 + diff_4) / 4
            weights.append(mean_diff)
        self.weights = weights

        # Normalize weights
        self.weights /= np.sum(self.weights)


    def resample(self):
        particles = []
        #particles = np.array([[0,0,0]])

        
        #norm_weights = weights/np.sum(weights)
        
        indices = [np.random.choice(np.arange(0, self.num_particles), p=self.weights) for i in range(self.num_particles)]

        for i in indices:
            particles.append(self.list_test[i])
            #particles = np.append(particles, self.particles[i])
        assert self.num_particles == len(particles)

        self.particles = particles
        self.num_particles = len(particles)

        # msg = Float64()
        # msg.data = num_particles
        # self.particle_count_pub.publish(msg)


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            
            self.weighting() 
            #self.resample()




            msg = Point()
            mean_pos_x = 0
            mean_pos_y = 0
            mean_pos_z = 0

            for i in range(self.num_particles):
                mean_pos_x += self.particles[i][0]
                mean_pos_y += self.particles[i][1]
                mean_pos_z += self.particles[i][2]

            msg.x = mean_pos_x / self.num_particles
            msg.y = mean_pos_y / self.num_particles
            msg.z = mean_pos_z / self.num_particles
    
            self.position_pub.publish(msg)


            rate.sleep()


def main():
    node = LocalizationNode()
    node.run()

if __name__ == "__main__":
    main()
