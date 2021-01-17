#!/usr/bin/env python
import rospy
import random
import math
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from range_sensor.msg import RangeMeasurementArray


TANK_DEPTH = -1
TANK_LENGTH = 4
TANK_WIDTH = 2

SENSOR_POSITION = np.array([0, 0.2, 0.1])

PARTICLE_COUNT = 1000

class LocalizationNode():
    def __init__(self):
        rospy.init_node("localization")

        # Simulation
        self.tag_1 = np.array([0.5, 3.35, -0.5])
        self.tag_2 = np.array([1.1, 3.35, -0.5])
        self.tag_3 = np.array([0.5, 3.35, -0.9])
        self.tag_4 = np.array([1.1, 3.35, -0.9])

        # Experiment
        # self.tag_1 = np.array([0.5, 3.35, -0.28])
        # self.tag_2 = np.array([1.1, 3.35, -0.28])
        # self.tag_3 = np.array([0.5, 3.35, -0.68])
        # self.tag_4 = np.array([1.1, 3.35, -0.68])

        self.num_particles = PARTICLE_COUNT
        
        self.particles = []
        self.weights = []

        self.ranges = np.array([0.0, 0.0, 0.0, 0.0])

        self.initialize_particles()

        self.position_pub = rospy.Publisher("mcl_position", Point, queue_size=1)
        self.error_pub = rospy.Publisher("error_mcl_prediction", Float64, queue_size=1)

        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback, queue_size=1)


    def range_callback(self, msg):
        num_measurements = len(msg.measurements)
        if num_measurements:
            for measurement in msg.measurements:
                self.ranges[measurement.id - 1] = measurement.range
        else:
            print("WARNING: No measurements received!")


    def initialize_particles(self):
        for i in range(self.num_particles):
            x = round(random.random(), 4) * TANK_LENGTH
            y = round(random.random(), 4) * TANK_WIDTH
            z = round(random.random(), 4) * TANK_DEPTH
            self.particles.append(np.array([x, y, z]))

            # self.particles[n] returns n'th particle
            # self.particles[n][m] returns m'th coordinate of nth particle


    def dead_reckon(self):
        pass


    def motion_model(self):
        particles = []
        sigma = 0.05
        for i, particle in enumerate(self.particles):
            p1 = random.gauss(self.particles[i][0], sigma)
            p2 = random.gauss(self.particles[i][1], sigma)
            p3 = random.gauss(self.particles[i][2], sigma)
            particles.append(np.array([p1, p2, p3]))

        self.particles = particles


    def measurement_model(self):
        weights = []
        # Dont make sigma too small
        sigma = 0.05

        for i, particle in enumerate(self.particles):
            diff1 = np.linalg.norm(self.particles[i] -
                                   self.tag_1) - self.ranges[0]
            diff2 = np.linalg.norm(self.particles[i] -
                                   self.tag_2) - self.ranges[1]
            diff3 = np.linalg.norm(self.particles[i] -
                                   self.tag_3) - self.ranges[2]
            diff4 = np.linalg.norm(self.particles[i] -
                                   self.tag_4) - self.ranges[3]
            w1 = abs((1 / sigma * np.sqrt(2 * math.pi)) *
                     np.exp(-1 / 2 * np.power(diff1 / sigma, 2.0))) + 0.000001
            w2 = abs((1 / sigma * np.sqrt(2 * math.pi)) *
                     np.exp(-1 / 2 * np.power(diff2 / sigma, 2.0))) + 0.000001
            w3 = abs((1 / sigma * np.sqrt(2 * math.pi)) *
                     np.exp(-1 / 2 * np.power(diff3 / sigma, 2.0))) + 0.000001
            w4 = abs((1 / sigma * np.sqrt(2 * math.pi)) *
                     np.exp(-1 / 2 * np.power(diff4 / sigma, 2.0))) + 0.000001

            weights.append(np.mean([w1, w2, w3, w4]))


        # Normalize weights
        weight_sum = np.sum(weights)
        if weight_sum > 0:
            weights /= weight_sum
            self.weights = weights
        else:
            print("WARNING: Weight sum is zero! Normalization not possible!")


    def resample(self):
        particles = []
        index_array = range(self.num_particles)

        for i in index_array:
            index = np.random.choice(index_array, p=self.weights)
            particles.append(self.particles[index])

        self.particles = particles


    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.motion_model()
            self.measurement_model()
            self.resample()

            particle_mean = np.mean(self.particles, axis=0)
            particle_mean -= SENSOR_POSITION
            particle_variance = np.var(self.particles, axis=0)

            position_msg = Point()
            position_msg.x = particle_mean[0]
            position_msg.y = particle_mean[1]
            position_msg.z = particle_mean[2]
            error_msg = Float64()
            error_msg.data = np.mean(particle_variance)

            self.position_pub.publish(position_msg)
            self.error_pub.publish(error_msg)

            rate.sleep()


def main():
    node = LocalizationNode()
    node.run()

if __name__ == "__main__":
    main()
