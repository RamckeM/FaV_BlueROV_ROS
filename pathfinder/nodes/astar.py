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
        rospy.init_node("astar")

        self.parent = dict()
        self.g = dict()

        self.OPEN = []
        self.CLOSED = []

        self.mcl_position = Point()
        self.goal_position = Point()
        self.start_position = Point()

        # Threshold from when an obstacle is assumed to exist
        self.threshold = rospy.get_param('~threshold', '0.5')

        # Additional safety distance which is not to be exceeded
        self.safety_distance = rospy.get_param('~safety_distance', '1.0')

# TBD
        self.map = np.array([35][70])


        # Listens to manual published topic
        self.goal_position_sub = rospy.Subscriber("goal_position", Point, self.goal_position_callback, queue_size=1)

        self.mcl_position_sub = rospy.Subscriber("mcl_position", Point, self.mcl_position_callback, queue_size=1)


# Interface TBD!
        #self.static_map_sub = rospy.Subscriber("static_map", )


# Change position_controller subscriber!
        self.waypoint_pub = rospy.Publisher("waypoints", WaypointArray, queue_size=1)



    def goal_position_callback(self, msg):
        self.goal_position.x = msg.x
        self.goal_position.y = msg.y
        self.goal_position.z = msg.z

    def mcl_position_callback(self, msg):
        self.mcl_position.x = msg.x
        self.mcl_position.y = msg.y
        self.mcl_position.z = msg.z

# WIP
    #def static_map_callback(self, msg):
        #self.map = msg



    # Checks the euclidean distance from current node to goal
    def heuristic(self, node):
        return math.hypot(self.goal_position.x - node.x, self.goal_position.y - node.y)


    # Assigns a cost to given node based on the additive cost of the last known node and the heuristic
    def f_value(self, node):
        return self.g[node] + self.heuristic(node)


    # Assigns a discrete, integer grid value to a given continuous point
    def assign_grid_position(self, point):
        int_point = Point()
        int_point.x = int(round(point.x))
        int_point.y = int(round(point.y))
        int_point.z = point.z
        return int_point


    # Returns all neighbors of the given node
# Bessere Alternative?
    def get_neighbor(self, node):
        neighbors = []
        neighbor = Point()
        neighbor.z = 0

        neighbor.x = node.x + 1
        neighbor.y = node.y + 1
        neighbors.append(neighbor)

        neighbor.x = node.x + 1
        neighbor.y = node.y - 1
        neighbors.append(neighbor)

        neighbor.x = node.x + 1
        neighbor.y = node.y
        neighbors.append(neighbor)

        neighbor.x = node.x - 1
        neighbor.y = node.y + 1
        neighbors.append(neighbor)

        neighbor.x = node.x - 1
        neighbor.y = node.y - 1
        neighbors.append(neighbor)

        neighbor.x = node.x - 1
        neighbor.y = node.y
        neighbors.append(neighbor)

        neighbor.x = node.x
        neighbor.y = node.y + 1
        neighbors.append(neighbor)

        neighbor.x = node.x
        neighbor.y = node.y - 1
        neighbors.append(neighbor)

        return neighbors



    # Checks for a collision for the given node
    def check_collision(self, node):
# Wird die safety_distance weggerundet?
# Alternativ: map[node.x + 1][node.y + 1] etc. -> Eine Zelle safety_distance
        if(self.map[int(round(node.x))][int(round(node.y))] > self.threshold or 
           self.map[int(round(node.x + self.safety_distance))][int(round(node.y + self.safety_distance))] > self.threshold or
           self.map[int(round(node.x - self.safety_distance))][int(round(node.y + self.safety_distance))] > self.threshold or
           self.map[int(round(node.x + self.safety_distance))][int(round(node.y - self.safety_distance))] > self.threshold or
           self.map[int(round(node.x - self.safety_distance))][int(round(node.y - self.safety_distance))] > self.threshold):
            return True

        return False


    # Determines the cost/distance between two given nodes
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
        while point is not self.start_position:
            point = self.parent[point]
            path.append(point)

# DEBUG
        for point in path:
            rospy.loginfo("%d %d", point.x, point.y)

        return path


    def search(self):
        self.parent[self.start_position] = self.start_position
        self.g[self.start_position] = 0
        self.g[self.goal_position] = math.inf

        heapq.heappush(self.OPEN, (self.f_value(self.start_position), self.start_position))

        while self.OPEN:
            node = heapq.heappop(self.OPEN)

            if node == self.assign_grid_position(self.goal_position):
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
            self.start_postion = self.assign_grid_position(self.mcl_position)

            # A* algorithm
            self.search()


            path_msg = WaypointArray()
            waypoint_msg = Waypoint()
            index = 1
            for waypoint in self.generate_path():
                waypoint_msg.id = index
                waypoint_msg.point = waypoint
                path_msg.waypoints.append(waypoint_msg)
                index += 1
            
# Setze z-Koordinate auf const. Wert

            self.waypoint_pub.publish(path_msg)

            rate.sleep()


def main():
    node = PathPlanningNode()
    node.run()

if __name__ == "__main__":
    main()
