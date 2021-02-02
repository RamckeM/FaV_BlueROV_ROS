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
        rospy.init_node("test_astar")

        self.start_position = Point()
        self.goal_position = Point()

        self.OPEN = []
        self.CLOSED = []

        self.PARENT = dict()
        self.g = dict()


        # Threshold from when an obstacle is assumed to exist
        self.threshold = rospy.get_param('~threshold', '0.5')

        self.map_resolution_x = rospy.get_param('/map_resolution_x')
        self.map_resolution_y = rospy.get_param('/map_resolution_y')

        # Initialize empty map
        self.map = np.zeros((self.map_resolution_y, self.map_resolution_x))

        self.mapping_sub = rospy.Subscriber("mapping", Floats, self.mapping_callback, queue_size=(self.map_resolution_x * self.map_resolution_y))

        self.waypoint_pub = rospy.Publisher("waypoints", WaypointArray, queue_size=8)

    
    def mapping_callback(self, msg):
        self.map = np.reshape(np.array((msg.data)), (self.map_resolution_y, self.map_resolution_x))



    # Checks the euclidean distance from current node to goal
    def heuristic(self, node):
        return math.hypot(self.goal_position.x - node.x, self.goal_position.y - node.y)


    # Assigns a cost to given node based on the additive cost of the last known node and the heuristic
    def f_value(self, node):
        return self.g[node] + self.heuristic(node)
    

    def get_neighbors(self, node):
        # neighbors = []
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

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    1")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        if node.x < self.map_resolution_x and node.y > 0:
            neighbor.x = node.x + 1
            neighbor.y = node.y - 1

            neighbor2.x = neighbor.x
            neighbor2.y = neighbor.y
            neighbor2.z = neighbor.z

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    2")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        if node.x < self.map_resolution_x:
            neighbor.x = node.x + 1
            neighbor.y = node.y

            neighbor3.x = neighbor.x
            neighbor3.y = neighbor.y
            neighbor3.z = neighbor.z

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    3")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        if node.x > 0 and node.y < self.map_resolution_y:
            neighbor.x = node.x - 1
            neighbor.y = node.y + 1

            neighbor4.x = neighbor.x
            neighbor4.y = neighbor.y
            neighbor4.z = neighbor.z

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    4")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        if node.x > 0 and node.y > 0:
            neighbor.x = node.x - 1
            neighbor.y = node.y - 1

            neighbor5.x = neighbor.x
            neighbor5.y = neighbor.y
            neighbor5.z = neighbor.z

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    5")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        if node.x > 0:
            neighbor.x = node.x - 1
            neighbor.y = node.y

            neighbor6.x = neighbor.x
            neighbor6.y = neighbor.y
            neighbor6.z = neighbor.z

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    6")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        if node.y < self.map_resolution_y:
            neighbor.x = node.x
            neighbor.y = node.y + 1

            neighbor7.x = neighbor.x
            neighbor7.y = neighbor.y
            neighbor7.z = neighbor.z

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    7")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        if node.y > 0:
            neighbor.x = node.x
            neighbor.y = node.y - 1

            neighbor8.x = neighbor.x
            neighbor8.y = neighbor.y
            neighbor8.z = neighbor.z

            # neighbors.append(neighbor)
            # rospy.loginfo("CHECK    8")
            # rospy.loginfo(neighbor)
            # rospy.loginfo("LIST:")
            # rospy.loginfo(neighbors)

        neighbors = (neighbor1, neighbor2, neighbor3, neighbor4, neighbor5, neighbor6, neighbor7, neighbor8)
        # rospy.loginfo("Neighbors:")
        # rospy.loginfo(neighbors)
        return neighbors


    def check_collision(self, node):
        if(self.map[int(round(node.y)), int(round(node.x))] > self.threshold):
            return True

        return False

    
    def cost(self, node1, node2):
        if self.check_collision(node2):
            return np.inf
        
        return math.hypot(node2.x - node1.x, node2.y - node1.y)


    def generate_path(self):
        path = [self.goal_position]
        q = self.goal_position

        key_list = self.PARENT.keys()

        rospy.loginfo("################################")
        rospy.loginfo("################################")
        rospy.loginfo(q)

        for key in key_list:
            if key == q:
                q = self.PARENT[key]
                rospy.loginfo(q)
                rospy.loginfo(self.PARENT[key])
                break
        
        q.x = self.test1.x
        q.y = self.test1.y
        q.z = self.test1.z
        rospy.loginfo("test1")
        rospy.loginfo(q)
        for key in key_list:
            if key == q:
                q = self.PARENT[key]
                rospy.loginfo(q)
                rospy.loginfo(self.PARENT[key])
                break
        
        q.x = self.test2.x
        q.y = self.test2.y
        q.z = self.test2.z
        rospy.loginfo("test2")
        rospy.loginfo(q)
        for key in key_list:
            if key == q:
                q = self.PARENT[key]
                rospy.loginfo(q)
                rospy.loginfo(self.PARENT[key])
                break
        
        q.x = self.test3.x
        q.y = self.test3.y
        q.z = self.test3.z
        rospy.loginfo("test3")
        rospy.loginfo(q)
        for key in key_list:
            if key == q:
                q = self.PARENT[key]
                rospy.loginfo(q)
                rospy.loginfo(self.PARENT[key])
                break

        q.x = self.test4.x
        q.y = self.test4.y
        q.z = self.test4.z
        rospy.loginfo("test4")
        rospy.loginfo(q)
        for key in key_list:
            if key == q:
                q = self.PARENT[key]
                rospy.loginfo(q)
                rospy.loginfo(self.PARENT[key])
                break

        q.x = self.test5.x
        q.y = self.test5.y
        q.z = self.test5.z
        rospy.loginfo("test5")
        rospy.loginfo(q)
        for key in key_list:
            if key == q:
                q = self.PARENT[key]
                rospy.loginfo(q)
                rospy.loginfo(self.PARENT[key])
                break





# That shit ain't deterministic

        # for key in key_list:
        #     if key == q:
        #         q = PARENT[key]
        # rospy.loginfo("NODE 1:")
        # rospy.loginfo(q)

        # for key in key_list:
        #     if key == q:
        #         q = PARENT[key]
        # rospy.loginfo("NODE 2")
        # rospy.loginfo(q)

        # while True:
        #     for key in key_list:
        #         if key == q:
        #             q = PARENT[key]
        #             path.append(q)

        #     if q == self.start_position:
        #         break

        # return list(path)



    def search(self):
        # Reset
        self.OPEN = []
        self.CLOSED = []
        self.PARENT.clear()
        self.g.clear()

        self.PARENT[self.start_position] = self.start_position
        self.g[self.start_position] = 0
        self.g[self.goal_position] = np.inf

        heapq.heappush(self.OPEN, (self.f_value(self.start_position), self.start_position))

        while self.OPEN:

            _, q = heapq.heappop(self.OPEN)
            self.CLOSED.append(q)
#DEBUG
            # rospy.loginfo(q)
            # rospy.loginfo(self.PARENT[q])

            if q == self.goal_position:
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
        rospy.loginfo(self.PARENT)


    def run(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():

            self.start_position.x = 1
            self.start_position.y = 1
            self.start_position.z = 0

            self.goal_position.x = 8
            self.goal_position.y = 8
            self.goal_position.z = 0

            self.test1=Point()
            self.test1.x = 6
            self.test1.y = 6
            self.test1.z = 0

            self.test2=Point()
            self.test2.x = 5
            self.test2.y = 5
            self.test2.z = 0

            self.test3=Point()
            self.test3.x = 4
            self.test3.y = 4
            self.test3.z = 0

            self.test4=Point()
            self.test4.x = 3
            self.test4.y = 3
            self.test4.z = 0

            self.test5=Point()
            self.test5.x = 2
            self.test5.y = 2
            self.test5.z = 0



            #path, visited = self.search()
            self.search()

            # path_msg = WaypointArray()
            # waypoint_msg = Waypoint()
            # # Dirty
            # for i, waypoint in enumerate(self.CLOSED):
            #     waypoint_msg.id = i
            #     waypoint_msg.point = waypoint
            #     path_msg.waypoints.append(waypoint_msg)

            # self.waypoint_pub.publish(path_msg)

            rate.sleep()



def main():
    node = PathPlanningNode()
    node.run()

if __name__ == "__main__":
    main()


