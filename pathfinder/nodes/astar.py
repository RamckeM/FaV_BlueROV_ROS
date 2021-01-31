#!/usr/bin/env python
import rospy
import math
import heapq
import numpy as np
from geometry_msgs.msg import Point
from rospy_tutorials.msg import Floats
# Custom message for publishing an array of geometry.point
from pathfinder.msg import Waypoint, WaypointArray


class PathPlanningNode():
    def __init__(self):
        rospy.init_node("astar")

        self.PARENT = dict()
        self.g = dict()

        self.OPEN = []
        self.CLOSED = []

        self.mcl_position = Point()
        self.start_position = Point()
        self.goal_position = Point()

        # Threshold from when an obstacle is assumed to exist
        self.threshold = rospy.get_param('~threshold', '0.5')

        # Additional safety distance which is not to be exceeded
        self.safety_distance = rospy.get_param('~safety_distance', '1.0')

        self.map_resolution_x = rospy.get_param('/map_resolution_x')
        self.map_resolution_y = rospy.get_param('/map_resolution_y')

        # Initialize empty map
        self.map = np.zeros((self.map_resolution_y, self.map_resolution_x))

        # Listens to manual published topic
        self.goal_position_sub = rospy.Subscriber("goal_position", Point, self.goal_position_callback, queue_size=1)

        self.mcl_position_sub = rospy.Subscriber("mcl_position", Point, self.mcl_position_callback, queue_size=1)

        self.mapping_sub = rospy.Subscriber("mapping", Floats, self.mapping_callback, queue_size=(self.map_resolution_x * self.map_resolution_y))

#TODO: Change position_controller subscriber!
        self.waypoint_pub = rospy.Publisher("waypoints", WaypointArray, queue_size=1)



    def goal_position_callback(self, msg):
        self.goal_position.x = msg.x
        self.goal_position.y = msg.y
        self.goal_position.z = msg.z

    def mcl_position_callback(self, msg):
        self.mcl_position.x = msg.x
        self.mcl_position.y = msg.y
        self.mcl_position.z = msg.z

    def mapping_callback(self, msg):
        self.map = np.reshape(np.array((msg.data)), (self.map_resolution_y, self.map_resolution_x))


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
    # Ugly af, don't read!
    def get_neighbor(self, node):
        neighbors = []
        neighbor = Point()
        neighbor.z = 0

        if node.x < self.map_resolution_x and node.y < self.map_resolution_y:
            neighbor.x = node.x + 1
            neighbor.y = node.y + 1
            neighbors.append(neighbor)

        if node.x < self.map_resolution_x and node.y > 0:
            neighbor.x = node.x + 1
            neighbor.y = node.y - 1
            neighbors.append(neighbor)

        if node.x < self.map_resolution_x:
            neighbor.x = node.x + 1
            neighbor.y = node.y
            neighbors.append(neighbor)

        if node.x > 0 and node.y < self.map_resolution_y:
            neighbor.x = node.x - 1
            neighbor.y = node.y + 1
            neighbors.append(neighbor)

        if node.x > 0 and node.y > 0:
            neighbor.x = node.x - 1
            neighbor.y = node.y - 1
            neighbors.append(neighbor)

        if node.x > 0:
            neighbor.x = node.x - 1
            neighbor.y = node.y
            neighbors.append(neighbor)

        if node.y < self.map_resolution_y:
            neighbor.x = node.x
            neighbor.y = node.y + 1
            neighbors.append(neighbor)

        if node.y > 0:
            neighbor.x = node.x
            neighbor.y = node.y - 1
            neighbors.append(neighbor)

        return neighbors


    # Checks for a collision for the given node
    def check_collision(self, node):
# Wird die safety_distance weggerundet?
        # if(self.map[int(round(node.x)), int(round(node.y))] > self.threshold or 
        #    self.map[int(round(node.x + self.safety_distance)), int(round(node.y + self.safety_distance))] > self.threshold or
        #    self.map[int(round(node.x - self.safety_distance)), int(round(node.y + self.safety_distance))] > self.threshold or
        #    self.map[int(round(node.x + self.safety_distance)), int(round(node.y - self.safety_distance))] > self.threshold or
        #    self.map[int(round(node.x - self.safety_distance)), int(round(node.y - self.safety_distance))] > self.threshold):
        if(self.map[int(round(node.y)), int(round(node.x))] > self.threshold):
            return True

        return False


    # Determines the cost/distance between two given nodes
    # Used to calculate the costs for corresponding neighbor nodes
    def cost(self, node1, node2):

        if self.check_collision(node2):
            return np.inf
        
        return math.hypot(node2.x - node1.x, node2.y - node1.y)


    # Will be called in main after finished search function
    def generate_path(self):
        path = [self.goal_position]
        s = self.goal_position
        
        # while s is not self.start_position:
        #     s = self.PARENT[s]
        #     path.append(s)

        rospy.loginfo(len(self.PARENT)) # für 10x10 matrix: 3013
            #rospy.loginfo(len(PARENT)) 

# DEBUG
        # for point in path:
        #     rospy.loginfo("%d %d", point.x, point.y)

        return list(path)


    def search(self):
        self.PARENT[self.start_position] = self.start_position
        self.g[self.start_position] = 0
        self.g[self.goal_position] = np.inf

        heapq.heappush(self.OPEN, (self.f_value(self.start_position), self.start_position))

        while self.OPEN:
            _, node = heapq.heappop(self.OPEN)

            if node == self.assign_grid_position(self.goal_position):
                break

            for neighbor in self.get_neighbor(node):
                new_cost = self.g[node] + self.cost(node, neighbor)

                if neighbor not in self.g:
                    self.g[neighbor] = np.inf

                if new_cost < self.g[neighbor]:
                    self.g[neighbor] = new_cost
                    self.PARENT[neighbor] = node
                    heapq.heappush(self.OPEN, (self.f_value(neighbor), neighbor))

    

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():

            # Fix the start position for one iteration
            self.start_postion = self.assign_grid_position(self.mcl_position)

#DEBUG
            self.start_position.x = 1
            self.start_position.y = 1
#DEBUG
            self.goal_position.x = 8
            self.goal_position.y = 8


            # A* algorithm
            self.search()


            self.generate_path()


            # path_msg = WaypointArray()
            # waypoint_msg = Waypoint()
            # index = 1
            # for waypoint in self.generate_path():
            #     waypoint_msg.id = index
            #     waypoint_msg.point = waypoint
            #     path_msg.waypoints.append(waypoint_msg)
            #     index += 1
            
#TODO: Setze z-Koordinate auf const. Wert

            self.waypoint_pub.publish(path_msg)

            rate.sleep()


def main():
    node = PathPlanningNode()
    node.run()

if __name__ == "__main__":
    main()
