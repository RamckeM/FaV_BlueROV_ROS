#!/usr/bin/env python
import rospy
import numpy as np
import random
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement


TANK_HEIGHT = 0
TANK_LENGTH = 0
TANK_WIDTH = 0

PARTICLE_COUNT = 200
ITERATION_STEPS = 20
PRECISION = 0.1


class LocalizationNode():
    def __init__(self):
        rospy.init_node("localization")

        self.tag1 = Point(0.5, 3.35, -0.5)
        self.tag2 = Point(1.1, 3.35, -0.5)
        self.tag3 = Point(0.5, 3.35, -0.9)
        self.tag4 = Point(1.1, 3.35, -0.9)

        self.particles = []

        self.num_particles = PARTICLE_COUNT
        self.steps = ITERATION_STEPS
        self.precision = PRECISION

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        

        self.particle_count_pub = rospy.Publisher("particle_count", Float64, queue_size=1)
        self.point_pub = rospy.Publisher("estimated_point", Point, queue_size=1)
        self.yaw_pub = rospy.Publisher("yaw", Float64, queue_size=1)


        self.range = 0
        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback, queue_size=1)
        self.orientation_sub = rospy.Subscriber("mavros/local_position/pose", Pose, self.orientation_callback, queue_size=1)
    

    def range_callback(self, msg):
# Get data correctly
        self.range[1] = msg.range
        self.range[2] = msg.range
        self.range[3] = msg.range
        self.range[4] = msg.range

        #Turn till three sensor measurements are valid


    
    def orientation_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.roll = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
        self.pitch = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)
        self.yaw = math.asin(2*x*y + 2*z*w)        


    def move(self):
        pass


    def compare_sensor_data(self):
        weights = []

        for i in self.particles:
            point1 = Point()
            point1.position.x = i[0]
            point1.position.y = i[1]
            point1.position.z = i[2]



            


    def resample(self, weights):
        particles = []

        # Normalize weights
        norm_weights = weights/np.sum(weights)
        
        indices = [np.random.choice(np.arange(0, self.num_particles), p=norm_weights) for i in range(self.num_particles)]

        for i in indices:
            particles.append(self.particles[i])
        assert self.num_particles == len(particles)

        self.particles = particles
        self.num_particles = len(particles)

        msg = Float64()
        msg.data = num_particles
        self.particle_count_pub.publish(msg)


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            # Initialize particles
            for i in range(self.num_particles):
                x = round(random.random(), 4) * self.tank_length
                y = round(random.random(), 4) * self.tank_width
                z = round(random.random(), 4) * self.tank_high
                self.particles.append((x, y, z))

            # Monte Carlo Localization
            for i in range(self.steps):
                self.move()
                self.calc()
                self.resample()

                
            
            msg = Point()
            mean_pos_x = 0
            mean_pos_y = 0
            mean_pos_z = 0
            for i in self.particles:
                mean_pos_x += i[0]
                mean_pos_y += i[1]
                mean_pos_z += i[2]

            msg.position.x = mean_pos_x / self.num_particles
            msg.position.y = mean_pos_y / self.num_particles
            msg.position.z = mean_pos_z / self.num_particles
    
        self.point_pub.publish(msg)

        rate.sleep()


def main():
    node = LocalizationNode()
    node.run()

if __name__ == "__main__":
    main()
