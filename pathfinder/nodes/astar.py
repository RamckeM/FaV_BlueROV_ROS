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
#TODO: Subscribe to knew estimation
        self.mcl_position_sub = rospy.Subscriber("mcl_position", Point, self.mcl_position_callback, queue_size=1)

        self.mapping_sub = rospy.Subscriber("mapping", Floats, self.mapping_callback, queue_size=(self.map_resolution_x * self.map_resolution_y))

#TODO: Change position_controller subscriber!
        self.waypoint_array_pub = rospy.Publisher("waypoints", WaypointArray, queue_size=50)
        self.waypoint_pub = rospy.Publisher("position_setpoint", Point, queue_size=1)



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
#TODO: Could use this function to translate position
    def assign_grid_position(self, point):
        int_point = Point()
        int_point.x = int(round(point.x))
        int_point.y = int(round(point.y))
        int_point.z = point.z
        return int_point


    # Returns all neighbors of the given node
    def get_neighbors(self, node):
        neighbor = Point()
        neighbor.z = 0

        neighbor1 = Point()
        neighbor1.x = node.x
        neighbor1.y = node.y
        neighbor1.z = node.z

        neighbor2 = Point()
        neighbor2.x = node.x
        neighbor2.y = node.y
        neighbor2.z = node.z

        neighbor3 = Point()
        neighbor3.x = node.x
        neighbor3.y = node.y
        neighbor3.z = node.z

        neighbor4 = Point()
        neighbor4.x = node.x
        neighbor4.y = node.y
        neighbor4.z = node.z
        
        neighbor5 = Point()
        neighbor5.x = node.x
        neighbor5.y = node.y
        neighbor5.z = node.z

        neighbor6 = Point()
        neighbor6.x = node.x
        neighbor6.y = node.y
        neighbor6.z = node.z

        neighbor7 = Point()
        neighbor7.x = node.x
        neighbor7.y = node.y
        neighbor7.z = node.z

        neighbor8 = Point()
        neighbor8.x = node.x
        neighbor8.y = node.y
        neighbor8.z = node.z
        
        if node.x < self.map_resolution_x and node.y < self.map_resolution_y:
            neighbor.x = node.x + 1
            neighbor.y = node.y + 1

            neighbor1.x = neighbor.x
            neighbor1.y = neighbor.y
            neighbor1.z = neighbor.z

        if node.x < self.map_resolution_x and node.y > 0:
            neighbor.x = node.x + 1
            neighbor.y = node.y - 1

            neighbor2.x = neighbor.x
            neighbor2.y = neighbor.y
            neighbor2.z = neighbor.z

        if node.x < self.map_resolution_x:
            neighbor.x = node.x + 1
            neighbor.y = node.y

            neighbor3.x = neighbor.x
            neighbor3.y = neighbor.y
            neighbor3.z = neighbor.z

        if node.x > 0 and node.y < self.map_resolution_y:
            neighbor.x = node.x - 1
            neighbor.y = node.y + 1

            neighbor4.x = neighbor.x
            neighbor4.y = neighbor.y
            neighbor4.z = neighbor.z

        if node.x > 0 and node.y > 0:
            neighbor.x = node.x - 1
            neighbor.y = node.y - 1

            neighbor5.x = neighbor.x
            neighbor5.y = neighbor.y
            neighbor5.z = neighbor.z

        if node.x > 0:
            neighbor.x = node.x - 1
            neighbor.y = node.y

            neighbor6.x = neighbor.x
            neighbor6.y = neighbor.y
            neighbor6.z = neighbor.z

        if node.y < self.map_resolution_y:
            neighbor.x = node.x
            neighbor.y = node.y + 1

            neighbor7.x = neighbor.x
            neighbor7.y = neighbor.y
            neighbor7.z = neighbor.z

        if node.y > 0:
            neighbor.x = node.x
            neighbor.y = node.y - 1

            neighbor8.x = neighbor.x
            neighbor8.y = neighbor.y
            neighbor8.z = neighbor.z

        neighbors = (neighbor1, neighbor2, neighbor3, neighbor4, neighbor5, neighbor6, neighbor7, neighbor8)

        return neighbors


    # Checks for a collision for the given node
    def check_collision(self, node):
# Wird die safety_distance weggerundet?

        if(self.map[int(round(node.x)), int(round(node.y))] > self.threshold or 
           self.map[int(round(node.x + self.safety_distance)), int(round(node.y + self.safety_distance))] > self.threshold or
           self.map[int(round(node.x - self.safety_distance)), int(round(node.y + self.safety_distance))] > self.threshold or
           self.map[int(round(node.x + self.safety_distance)), int(round(node.y - self.safety_distance))] > self.threshold or
           self.map[int(round(node.x - self.safety_distance)), int(round(node.y - self.safety_distance))] > self.threshold):
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
        pass
        # path = [self.goal_position]
        # q = self.goal_position
        
        # while s is not self.start_position:
        #     q = self.PARENT[q]
        #     path.append(q)

        # return list(path)


    def search(self):
        self.PARENT[self.start_position] = self.start_position
        self.g[self.start_position] = 0
        self.g[self.goal_position] = np.inf

        heapq.heappush(self.OPEN, (self.f_value(self.start_position), self.start_position))

        while self.OPEN:
            _, q = heapq.heappop(self.OPEN)
            self.CLOSED.append(q)

            if q == self.assign_grid_position(self.goal_position):
                break

            for neighbor in self.get_neighbors(q):
                new_cost = self.g[q] + self.cost(q, neighbor)

                if neighbor not in self.g:
                    self.g[neighbor] = np.inf

                if new_cost < self.g[neighbor]:
                    self.g[neighbor] = new_cost
                    self.PARENT[neighbor] = q
                    heapq.heappush(self.OPEN, (self.f_value(neighbor), neighbor))

        #return self.generate_path()
    

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():

            # Fix the start position for one iteration
            self.start_postion = self.assign_grid_position(self.mcl_position)

#####################################
            self.start_position.x = 1
            self.start_position.y = 1
            self.start_position.z = 0

            self.goal_position.x = 8
            self.goal_position.y = 8
            self.goal_position.z = 0
#####################################

            # Reset
            self.OPEN = []
            self.CLOSED = []
            self.PARENT.clear()
            self.g.clear()

            # A* algorithm
            self.search()


            waypoint_msg = Point()
            waypoint_msg.x = self.CLOSED[1].x
            waypoint_msg.y = self.CLOSED[1].y
            waypoint_msg.z = self.goal_position.z
            self.waypoint_pub.publish(waypoint_msg)
           

            path_msg = WaypointArray()
            waypoint2_msg = Waypoint()
            temp = []
            temp2 = []
            for i in range(len(self.CLOSED)):
#TODO: Point is reference and will be overwritten!
                waypoint2_msg.id = i
                waypoint2_msg.point.x = self.CLOSED[i].x
                waypoint2_msg.point.y = self.CLOSED[i].y
                waypoint2_msg.point.z = self.goal_position.z
                temp.append(waypoint2_msg.point)
            
            path_msg.waypoints = list(temp)
            self.waypoint_array_pub.publish(path_msg)


            rate.sleep()



def main():
    node = PathPlanningNode()
    node.run()

if __name__ == "__main__":
    main()
