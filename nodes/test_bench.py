#!/usr/bin/env python
from operator import gt
from numpy.lib.function_base import gradient
from numpy.ma.core import left_shift
import rospy
import time
from rospy.topics import Subscriber
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from std_msgs.msg import Float64
from rospy.numpy_msg import numpy_msg

TANK_Z = -1.
TANK_Y = 4.
TANK_X = 2.

TANK_NUMBER_Y = 70
TANK_NUMBER_X = 35

class MappingNode():
    def __init__(self):   
        # --- algorithm variables
        self.l_occ = 1.0
        self.l_free = -1.0
        self.l_0 = 0.0
        self.nb_cells = TANK_NUMBER_X*TANK_NUMBER_Y
        # --- columns: 1. x-coord, 2. y-coord, 3. weights
        self.grid = self.l_0 * np.ones((TANK_NUMBER_X*TANK_NUMBER_Y,3))
        # --- cell length and width
        self.cell_dx = TANK_X/TANK_NUMBER_X
        self.cell_dy = TANK_Y/TANK_NUMBER_Y
        # --- boundary data
        self.grid_boundary = np.array([[self.cell_dx/2, TANK_X - self.cell_dx/2], [self.cell_dy/2, TANK_Y - self.cell_dy/2]])
        # --- ROV
        self.position = Point()
        self.position.x = 1.
        self.position.y = 2.
        self.position.z = -0.5
        self.position_vec = np.array([self.position.x, self.position.y])
        self.yaw_gs = 0.5*math.pi # 0.  #ground state angle in pi!  # ATTTENTIONNN HAVE TO BE IN (0,2pi) !!1
        self.beta = math.pi/3 #angle of view
        self.alpha = np.array([0.2])
        self.radius_max = 1.
        # --- measurements
        self.obstacle = np.array([1., 2.75])
        # print(range(self.obstacle.size/2))

    def initialize_grid(self): 
        # --- initialize mesh
        x = np.linspace(0+self.cell_dx/2,TANK_X-self.cell_dx/2,TANK_NUMBER_X)
        y = np.linspace(0+self.cell_dy/2,TANK_Y-self.cell_dy/2,TANK_NUMBER_Y)
        xv, yv = np.meshgrid(x, y)
        xv = np.reshape(xv,TANK_NUMBER_X*TANK_NUMBER_Y)
        yv = np.reshape(yv,TANK_NUMBER_X*TANK_NUMBER_Y)
        # --- set coordinates
        self.grid[:,0] = xv
        self.grid[:,1] = yv
        # --- set weights of boundary near to 1
        factor = 100
        self.grid[0:TANK_NUMBER_X,2] = factor*self.l_occ
        self.grid[self.nb_cells -TANK_NUMBER_X: self.nb_cells,2] = factor*self.l_occ
        self.grid[0:self.nb_cells:TANK_NUMBER_X ,2] = factor* self.l_occ
        self.grid[TANK_NUMBER_Y-1:self.nb_cells:TANK_NUMBER_X,2] = factor* self.l_occ

    def map_updating(self):
        #for i in range(self.nb_cells):
        for i in range(TANK_NUMBER_X+1,self.nb_cells-TANK_NUMBER_X-1):
            self.grid[i, 2] = self.grid[i, 2] + self.inverse_sensor_model(i) - self.l_0
        self.post_processing()

    def inverse_sensor_model(self, index):
        dist_grid = np.linalg.norm(self.grid[index,(0,1)]-self.position_vec)
        phi = math.atan2((self.grid[index,1] - self.position_vec[1]), (self.grid[index,0] - self.position_vec[0]))
        l_update = self.l_free
        for sensor in range(self.obstacle.size/2):
            sensor_index = sensor
            #print(self.obstacle.size/2)
            #obstacle = np.array([self.obstacle[sensor_index], self.obstacle[sensor_index+1]])
            if (self.obstacle.size/2) >= 2:
                obstacle = self.obstacle[sensor_index]
                alpha = self.alpha[sensor_index]
            else:
                obstacle = self.obstacle
                alpha = self.alpha
            #print(self.obstacle)
            #print(obstacle)
            dist_obst = np.linalg.norm(obstacle-self.position_vec)      
            phi_obstacle = math.atan2((obstacle[1] - self.position_vec[1]), (obstacle[0] - self.position_vec[0])) 
            phi_range = np.arctan(alpha/(2*dist_obst))

            #adaptation of angle bc overflow
            if ((phi - self.yaw_gs)) >= (math.pi):
                phi = phi-2*math.pi
                phi_obstacle = phi_obstacle - 2*math.pi
                #phi_range = phi_range -2*math.pi
            if ((phi - self.yaw_gs)) <= -(math.pi):
                phi = phi+2*math.pi
                phi_obstacle = phi_obstacle + 2*math.pi
                #phi_range = phi_range +2*math.pi
            # updater
            #if dist_grid > dist_obst or abs(phi - self.yaw_gs) > self.beta/2:
            #if dist_grid > min([self.radius_max, dist_obst+self.alpha/2]) or abs(phi - self.yaw_gs) > self.beta/2:
            if dist_grid > self.radius_max or abs(phi - self.yaw_gs) > self.beta/2 \
             or (dist_grid > dist_obst+alpha/2 and (((phi)) >= (phi_obstacle - phi_range) and (phi) <= (phi_obstacle + phi_range))):
                l_update = max([self.l_0, l_update])
            elif (dist_obst <= self.radius_max) and np.linalg.norm(self.grid[index,(0,1)]-obstacle) < alpha/2: #abs(dist_grid-dist_obst) < self.alpha/2:
                l_update = max([self.l_occ, l_update])
            else:
                l_update = max([self.l_free, l_update])
        #if dist_grid > dist_obst+self.alpha/2 and (((phi) >= (phi_obstacle - phi_range) and (phi) <= (phi_obstacle + phi_range))):
         #   l_update = self.l_0
        return l_update

    def post_processing(self):
        factor = 100.
        #self.grid[0:TANK_NUMBER_X,2] = factor*self.l_occ
        #self.grid[self.nb_cells -TANK_NUMBER_X: self.nb_cells,2] = factor*self.l_occ
        self.grid[0:self.nb_cells:TANK_NUMBER_X ,2] = factor* self.l_occ
        self.grid[TANK_NUMBER_Y-1:self.nb_cells:TANK_NUMBER_X,2] = factor* self.l_occ

    def plotting(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        self.grid[:,2] = 1.0 - 1.0 / (1.0 + np.exp(self.grid[:,2]))
        ax.scatter(self.grid[:,0],self.grid[:,1],self.grid[:,2])
        ax.view_init(elev=90., azim=(0))
        plt.show()

    def surfer(self):
        x = np.linspace(0+self.cell_dx/2,TANK_X-self.cell_dx/2,TANK_NUMBER_X)
        y = np.linspace(0+self.cell_dy/2,TANK_Y-self.cell_dy/2,TANK_NUMBER_Y)
        xv, yv = np.meshgrid(x, y)
        self.grid[:,2] = 1.0 - 1.0 / (1.0 + np.exp(self.grid[:,2]))
        grid_mat = np.reshape(self.grid[:,2],(TANK_NUMBER_Y,TANK_NUMBER_X))
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        surf = ax.plot_surface(xv, yv, grid_mat, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
        fig.colorbar(surf, shrink=0.5, aspect=5)
        ax.view_init(elev=90., azim=(0))
        plt.show()


    def run(self):
        self.initialize_grid()
        steps = int(2*math.pi/self.beta)
        self.obstacle = np.array([[0.85,1.25],[1.0, 2.5],[0.25, 2.0],[1.5, 2.],[1.5,2.5]])
        self.alpha = np.array([[0.2],[0.4],[0.3],[0.35],[0.2]])
        self.position_vec = np.array([self.position.x, self.position.y])
        for j in range(3):
            y = -1 +j
            self.position_vec = np.array([self.position.x, self.position.y-y])
            for jj in range(2):
                for i in range(steps):
                    self.yaw_gs = (i*1./steps) * 2* math.pi 
                    #self.obstacle = np.array([1.25, 2.75])
                    #self.yaw_gs = math.pi/2
                    start_time = time.time()
                    self.map_updating()
                    print("--- %s seconds ---" % (time.time() - start_time))
        print("cell_props:")
        print(self.cell_dx,self.cell_dy)
        print("nb_cells")
        print(self.nb_cells)
        self.plotting()
        #self.surfer()
        


def main():
    node = MappingNode()
    node.run()
    

if __name__ == "__main__":
    main()
