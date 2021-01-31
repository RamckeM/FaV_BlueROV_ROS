#!/usr/bin/env python
import rospy
import numpy as np
from rospy_tutorials.msg import Floats



class MockupMapNode():
    def __init__(self):
        rospy.init_node("mockup_map")

        self.map_resolution_x = rospy.get_param('/map_resolution_x')
        self.map_resolution_y = rospy.get_param('/map_resolution_y')

        self.map = np.zeros((self.map_resolution_y, self.map_resolution_x))

        self.mapping_pub = rospy.Publisher("mapping", Floats, queue_size=(self.map_resolution_x * self.map_resolution_y))
        

    def init_map(self):
        i = 0
        while i < self.map_resolution_x:
            self.map[0, i] = 1
            self.map[self.map_resolution_y - 1, i] = 1
            i += 1

        i = 0
        while i < self.map_resolution_y:
            self.map[i, 0] = 1
            self.map[i, self.map_resolution_x - 1] = 1
            i += 1

        # i, j = 12, 27
        # while i < 22:
        #     while j < 47:
        #         self.map[j, i] = 1
        #         j += 1
        #     j = 27
        #     i += 1


    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():

            self.init_map()

            mappy_mc_mapface = []
            for x in self.map:
                for y in x:
                    mappy_mc_mapface.append(y)

            msg = Floats()
            msg.data = mappy_mc_mapface
            self.mapping_pub.publish(msg)

            rate.sleep()



def main():
    node = MockupMapNode()
    node.run()

if __name__ == "__main__":
    main()
