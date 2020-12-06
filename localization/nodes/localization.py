#!/usr/bin/env python
import rospy
import numpy as np
import random
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement


TANK_HEIGHT = 0
TANK_LENGTH = 0
TANK_WIDTH = 0

PARTICLE_COUNT = 100


class LocalizationNode():
    def __init__(self):
        rospy.init_node("localization")

        self.tag1 = Pose(0.5, 3.35, -0.5, 0.0, 0.0, 0.0, 0.0)
        self.tag2 = Pose(1.1, 3.35, -0.5, 0.0, 0.0, 0.0, 0.0)
        self.tag3 = Pose(0.5, 3.35, -0.9, 0.0, 0.0, 0.0, 0.0)
        self.tag4 = Pose(1.1, 3.35, -0.9, 0.0, 0.0, 0.0, 0.0)

        self.particles = []
        self.max_particles = PARTICLE_COUNT
        self.steps = 10

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        

        self.particle_count_pub = rospy.Publisher("particle_count", Float64, queue_size=1)
        self.pose_pub = rospy.Publisher("estimated_pose", Pose, queue_size=1)
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


    def calc(self):
        pass


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
            for i in self.max_particle:
                x = round(random.random(), 4) * self.tank_length
                y = round(random.random(), 4) * self.tank_width
                z = round(random.random(), 4) * self.tank_high
                orientation = round(random.random(), 4) 
                self.particles.append((x, y, z, orientation))

            for i in range(self.steps):
                
                self.move()
                self.calc()
                self.resample()

                
            
            msg = Pose()
            for i in self.particles:
                msg.position.x = 

        
        
        
        msg.position.y = 0
        msg.position.z = 0
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 0
        self.pose_pub.publish(msg)

        rate.sleep()


def main():
    node = LocalizationNode()
    node.run()

if __name__ == "__main__":
    main()
