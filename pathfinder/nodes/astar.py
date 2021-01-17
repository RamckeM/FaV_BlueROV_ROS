#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import Point


class PathPlanningNode():
    def __init__(self):

        self.parent = dict()
        self.cost = dict()


        self.mcl_position = Point()
        self.goal_position = Point()


        # Listens to manual published topic
        self.goal_position_sub = rospy.Subscriber("goal_position", Point, self.goal_position_callback, queue_size=1)

        self.mcl_position_sub = rospy.Subscriber("mcl_position", Point, self.mcl_position_callback, queue_size=1)

        # Interface TBD!
        #self.static_map_sub = rospy.Subscriber("static_map", )



# Change position_controller subscriber!
        self.waypoint_pub = rospy.Publisher("waypoint", Point, queue_size=1)



    def goal_position_callback(self, msg):
        self.goal_position.x = msg.x
        self.goal_position.y = msg.y
        self.goal_position.z = msg.z

    def mcl_position_callback(self, msg):
        self.mcl_position.x = msg.x
        self.mcl_position.y = msg.y
        self.mcl_position.z = msg.z

    #def static_map_callback(self, msg):



    def search(self):
        self.parent[self.mcl_position] = self.mcl_position





    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():

            rate.sleep()


def main():
    node = PathPlanningNode()
    node.run()

if __name__ == "__main__":
    main()
