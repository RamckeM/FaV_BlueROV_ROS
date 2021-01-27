#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from operator import gt
from numpy.lib.function_base import gradient
from numpy.ma.core import left_shift, reshape
import rospy
import time
from rospy.topics import Subscriber
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from fav_object_detection.msg import ObjectDetectionArray, ObjectDetection
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, \
    TwistWithCovarianceStamped,Point
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion
import pylab
from pathlib import Path
from numpy import savetxt
from numpy import asarray
import csv   


TANK_Z = -1.
TANK_Y = 4.
TANK_X = 2.

TANK_NUMBER_Y = 80
TANK_NUMBER_X = 40


class PlottingNode():
    def __init__(self):
        rospy.init_node("mapping_plot")
        self.nb_cells = TANK_NUMBER_X*TANK_NUMBER_Y
        self.map = np.zeros((TANK_NUMBER_Y,TANK_NUMBER_X))
        self.cell_dx = TANK_X/TANK_NUMBER_X
        self.cell_dy = TANK_Y/TANK_NUMBER_Y
        self.mapping_sub = rospy.Subscriber("mapping",(Floats),self.mapping_callback,queue_size=(self.nb_cells))
        self.map = 0.5*np.ones(self.nb_cells)

    def mapping_callback(self,msg):
        self.map = msg.data#,(TANK_NUMBER_Y,TANK_NUMBER_X) ##np.reshape(msg.data,(TANK_NUMBER_Y,TANK_NUMBER_X))

    def scatter(self):
        x = 1

    def run(self):
        time.sleep(9)
        rate = rospy.Rate(10.0)
        x = np.linspace(0+self.cell_dx/2,TANK_X-self.cell_dx/2,TANK_NUMBER_X)
        y = np.linspace(0+self.cell_dy/2,TANK_Y-self.cell_dy/2,TANK_NUMBER_Y)
        xv, yv = np.meshgrid(x, y)
        self.xv = np.reshape(xv,TANK_NUMBER_X*TANK_NUMBER_Y)
        self.yv = np.reshape(yv,TANK_NUMBER_X*TANK_NUMBER_Y)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(self.xv, self.yv, self.map)
        ax.view_init(elev=90., azim=(0))
        plt.ion()
        #fig.show()
        #f=open('/home/hendrik/fav/catkin_ws/test.dat','ab')
        while not rospy.is_shutdown():
            print(np.mean(self.map))
            #np.savetxt(f,self.map)
            #np.savetxt(f,np.array([1]))
            
            with open(r'/home/hendrik/fav/catkin_ws/src/mapping_package/data.csv', 'ab') as f:
                print("safe")
                writer = csv.writer(f)
                writer.writerow(np.reshape(self.map,(self.nb_cells,1)))
            rate.sleep()
           


def main():
    node = PlottingNode()
    node.run()

if __name__ == "__main__":
    main()
