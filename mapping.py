#! /usr/bin/env python3
# import rospy
import matplotlib.pyplot as plt
import numpy as np
from bresenham import bresenham
import plots

# Probabilities chosen by the user to define the occupancy values
P_occupied = 0.6
P_free = 0.3
P_prior = 0.5


def log_odds(p):
    '''Calculate the log-odd probability'''
    return np.log(p/(1-p))


def restore_p(l):
    '''Restore the probability from log-odds '''
    return 1-1 / (1+np.exp(l))


class Map():

    def __init__(self, xlim, ylim, resolution, p):

        # limits of the map
        self.xlim = xlim
        self.ylim = ylim
        self.resolution = resolution  # grid resolution in [m]

        # cell values from (x, y) 2-D map
        x = np.arange(xlim[0], xlim[1] + resolution, step=resolution)
        y = np.arange(ylim[0], ylim[1] + resolution, step=resolution)

        self.xsize = len(x)  # number of cells in x direction
        self.ysize = len(y)  # number of cells in y direction
        self.map_size = self.xsize*self.ysize  # area of the map

        self.alpha = 0.5  # thickness of the obstacle
        self.z_max = 10.0  # max reading distance from the laser scan

        # initial log-odd probability map matrix
        self.log_map = np.full((self.xsize, self.ysize), log_odds(p))

        # log probabilities to update the map
        self.l_occupied = log_odds(P_occupied)
        self.l_free = log_odds(P_free)

    def laser_scan_to_2D(self, z, angles, x, y, theta):
        '''
        Convert laser measurements to X-Y plane for mapping purposes
        @param distances: distance measurements from the laser
        @param angles: angle measurements from the laser
        @param x, y, theta: robot position
        '''
        x_distances = np.array([])
        y_distances = np.array([])

        for (d, angle) in zip(z, angles):
            x_distances = np.append(x_distances, x+d*np.cos(angle+theta))
            y_distances = np.append(y_distances, y+d*np.sin(angle+theta))

        return (x_distances, y_distances)

    def map_coordinates(self, x_continuous, y_continuous):
        '''
        Convert (x,y) continuous coordinates to discrete coordinates
        '''
        x = int((x_continuous - self.xlim[0]) / self.resolution)
        y = int((y_continuous - self.ylim[0]) / self.resolution)

        return (x, y)

    def calculate_map(self, z, angles, x, y, theta):
        """
        Computes the occupancy-grid map for a given sensor/robot data
        """

        # laser measurements in 2-D plane
        x_distances, y_distances = self.laser_scan_to_2D(
            z, angles, x, y, theta)

        # initial (x, y) for Bresenham's algorithm
        x1, y1 = self.map_coordinates(x, y)

        for (d_x, d_y, dist) in zip(x_distances, y_distances, z):

            # ending (x, y) for Bresenham's algorithm
            x2, y2 = self.map_coordinates(d_x, d_y)

            r = np.sqrt((x-d_x)**2+(y-d_y)**2)

            # all cells between the robot position to the laser hit cell are free
            for (x_bresenham, y_bresenham) in bresenham(Map, x1, y1, x2, y2):
                self.log_map[x_bresenham, y_bresenham] += self.l_free

            # obstacle cells hit from laser are occupied
            if dist < self.z_max and abs(r-dist) < self.alpha/2:
                self.log_map[x2, y2] += self.l_occupied

        return self.log_map


if __name__ == '__main__':
    # Test micro-simulator
    xlim = [-10, 10]
    ylim = [-10, 10]
    resolution = 0.1
    z = [2, 4, 3, 5]
    angles = [np.pi/12, np.pi/6, 3*np.pi/12, 4*np.pi/12]
    x = 2.5
    y = 2.5
    theta = 0
    # create 2-D occupancy grid map
    Map = Map(xlim, ylim, resolution, P_prior)

    ''' 1. probably cycle the calculate_map function when we have more sensor/robot data
        2. maybe get the robot's volume and occupy the neighbor cells as free
        3. represent with different colors: robot, laser, obstacles and free
        4. need to get data from rosbags
        5. need better representation of maps - legend of values probabilities 0 to 1
    '''

    # final occupancy grid map
    occupancy_map = Map.calculate_map(z, angles, x, y, theta)

    # plot results in plots.py

    #plt.imshow(restore_p(occupancy_map), 'Blues')
    # plt.show()
    plt.imshow(occupancy_map, 'Blues')
    plt.show()
