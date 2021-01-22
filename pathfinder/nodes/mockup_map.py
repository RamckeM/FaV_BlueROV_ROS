#!/usr/bin/env python
import rospy
import numpy as np



class MockupMapNode():
    def __init__(self):
        pass


def run(self):
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():

        rate.sleep()


def main():
    node = MockupMapNode()
    node.run()

if __name__ == "__main__":
    main()
