#!/usr/bin/env python
import rospy
import math
import heapq
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point



# Custom message for publishing an array of geometry.point
from pathfinder.msg import Waypoint, WaypointArray


class PathPlanningNode():
    def __init__(self):

        self.parent = dict()
        self.g = dict()

        self.OPEN = []
        self.CLOSED = []

        self.mcl_position = Point()
        self.goal_position = Point()
        self.start_position = Point()

# TBD
        #self.map


        # Listens to manual published topic
        self.goal_position_sub = rospy.Subscriber("goal_position", Point, self.goal_position_callback, queue_size=1)

        self.mcl_position_sub = rospy.Subscriber("mcl_position", Point, self.mcl_position_callback, queue_size=1)

        # Interface TBD!
        #self.static_map_sub = rospy.Subscriber("static_map", )


# Change position_controller subscriber!
        self.waypoint_pub = rospy.Publisher("waypoint", WaypointArray, queue_size=1)



    def goal_position_callback(self, msg):
        self.goal_position.x = msg.x
        self.goal_position.y = msg.y
        self.goal_position.z = msg.z

    def mcl_position_callback(self, msg):
        self.mcl_position.x = msg.x
        self.mcl_position.y = msg.y
        self.mcl_position.z = msg.z

    #def static_map_callback(self, msg):
        #self.map = msg



    # Checks the euclidean distance from current node to goal
    # Used to determine the next best step
    def heuristic(self, node):
        return math.hypot(self.goal_position.x - node.x, self.goal_position.y - node.y)

    # Assigns a cost to given node based on the additive cost of the last known node and the heuristic
    def f_value(self, node):
        return self.g[node] + self.heuristic(node)

    # Returns all neighbors of the given node
    def get_neighbor(self, node):
        pass

    # Checks for a collision for the given node
# Based on tolerance/certainty
    def check_collision(self, node):
        pass

    # Determines the cost/distance between two give nodes
    # Used to calculate the costs for corresponding neighbor nodes
    def cost(self, node1, node2):

        if self.check_collision(node2):
            return math.inf
        
        return math.hypot(node2.x - node1.x, node2.y - node1.y)

    # Will be called in main after finished search function
    # Returns the parent dict in reverse order
    def generate_path(self):
        path = [self.goal_position]
        point = self.goal_position
# Kann Probleme machen wenn mcl_position nicht fix ist. Definiere Startposition ggf. neu
        while point is not self.mcl_position:
            point = self.parent[point]
            path.append(point)

        return list(path)


    def search(self):
        self.parent[self.mcl_position] = self.mcl_position
        self.g[self.mcl_position] = 0
        self.g[self.goal_position] = math.inf

        heapq.heappush(self.OPEN, (self.f_value(self.mcl_position), self.mcl_position))

        while self.OPEN:
            node = heapq.heappop(self.OPEN)

            if node == self.goal_position:
                break

            for neighbor in self.get_neighbor(node):
                new_cost = self.g[node] + self.cost(node, neighbor)

                if neighbor not in self.g:
                    self.g[neighbor] = math.inf

                if new_cost < self.g[neighbor]:
                    self.g[neighbor] = new_cost
                    self.PARENT[neighbor] = node
                    heapq.heappush(self.OPEN, (self.f_value(neighbor), neighbor))

    

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():

            # Fix the start position for one iteration
            self.start_postion = self.mcl_position

            # A* algorithm
            self.search()

            # Return list of waypoints
            #waypoints = self.generate_path()


            waypoint_msg = Waypoint()
            msg = WaypointArray()
            index = 1
# Is this good practice?
            for waypoint in self.generate_path():
                waypoint_msg.id = index
                waypoint_msg.point = waypoint
                index += 1
            
            # setze z koordiante auf const. wert?

            self.waypoint_pub.publish()

            rate.sleep()


def main():
    node = PathPlanningNode()
    node.run()

if __name__ == "__main__":
    main()
