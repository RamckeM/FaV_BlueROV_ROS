#!/usr/bin/env python
import rospy


class PositionControlNode():
    def __init__(self):
        rospy.init_node("position_controller")


def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            pass

        rate.sleep()


def main():
    node = PositionControlNode()
    node.run()

if __name__ == "__main__":
    main()
