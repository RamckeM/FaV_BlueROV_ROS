#!/usr/bin/env python
import rospy
import numpy as np
import random
from geometry_msgs.msg import Pose
from range_sensor.msg import RangeMeasurementArray, RangeMeasurement


TANK_HEIGHT = 0
TANK_LENGTH = 0
TANK_WIDTH = 0

PARTICLE_COUNT = 100


class LocalizationNode():
    def __init__(self):
        rospy.init_node("monte_carlo.py")

        self.tag1 = Pose(0.5, 3.35, -0.5, 1.57079632679, -0.0, 0.0)
        self.tag2 = Pose(1.1, 3.35, -0.5, 1.57079632679, -0.0, 0.0)
        self.tag3 = Pose(0.5, 3.35, -0.9, 1.57079632679, -0.0, 0.0)
        self.tag4 = Pose(1.1, 3.35, -0.9, 1.57079632679, -0.0, 0.0)

        self.particles = []
        self.max_particle = PARTICLE_COUNT


        self.range = 0
        self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range_callback, queue_size=1)

    

    def range_callback(self, msg):
        self.range = msg.data
        


    def initialize(self):
        for i in self.max_particle:
            x = round(random.random(), 4) * self.tank_length
            y = round(random.random(), 4) * self.tank_width
            z = round(random.random(), 4) * self.tank_high
            orientation = round(random.random(), 4) 
            self.particles.append((x, y, z, orientation))


    def resample(self):
        pass


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            self.initialize()

        rate.sleep()


def main():
    node = LocalizationNode()
    node.run()

if __name__ == "__main__":
    main()
