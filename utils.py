#! /usr/bin/env python3
import matplotlib.pyplot as plt
import cv2 as cv2
from bagpy import bagreader
import pandas as pd
import mapping
import numpy as np
import math


def maximum_likelihood(_map):
    """
    Calculate the maximum likelihood map by clipping the occupancy grid map at 0.5 threshold.
    """
    free = np.where(_map < 0.5)
    _map[free] = 0
    occupied = (np.where(_map > 0.5))
    _map[occupied] = 1
    return _map


def restore_p(matrix):
    """ Restore probability matrix from log-odds."""
    return (1 - np.divide(1, 1+np.exp(matrix)))


def compare_maps(map1, map2):
    """ Function to compare two maps. 
    Evaluation metric is"""
    free1 = 0
    occupied1 = 0
    free2 = 0
    occupied2 = 0
    diff0 = 0
    diff1 = 0
    difference_map = map1-map2
    for i in range(len(map1)):
        for j in range(len(map1[0])):
            if map1[i][j] == 0:
                free1 += 1
            elif map1[i][j] == 1:
                occupied1 += 1
            if map2[i][j] == 0:
                free2 += 1
            elif map2[i][j] == 1:
                occupied2 += 1
            if difference_map[i][j] == 0:
                diff0 += 1
            elif difference_map[i][j] == 1 or difference_map[i][j] == -1:
                diff1 += 1

    print(free1)
    print(occupied1)
    print(free2)
    print(occupied2)
    print(free1/free2)
    print(occupied1/occupied2)
    print(diff0)
    print(diff1)


def plot_map(occupancy_map, resolution, xlim, ylim):
    """Plot a simple figure to represent the occupancy map """

    plt.imshow(occupancy_map, 'Blues')

    # To represent the x-y limits of the map
    plt.xlim([0, len(occupancy_map)-1])
    plt.ylim([0, len(occupancy_map[0])-1])
    locs, labels = plt.xticks()
    labels = [float(item)*resolution-xlim[1] for item in locs]
    plt.xticks(locs, labels)
    locs, labels = plt.yticks()
    labels = [float(item)*resolution-ylim[1] for item in locs]
    plt.yticks(locs, labels)

    plt.title("2-D Occupancy Grid Map", fontsize=14, fontweight='bold')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.colorbar()


def plot_trajectory(bag_file):
    """ Plot the trajectory (x, y) of the robot """

    b = bagreader(bag_file)
    pose = b.message_by_topic('/pose')
    posedf = pd.read_csv(pose)

    posedf.plot(x='pose.pose.position.x', y='pose.pose.position.y',
                xlabel='x [m]', ylabel='y [m]', label='Trajectory of the robot')
    # just for initial position of robot
    plt.plot(-5.47, 3.6, 'ro')
    # just to circle the change of direction of the robot
    circle1 = plt.Circle((-16.80, -2.75), 0.8, color='r', alpha=0.3)
    plt.gca().add_patch(circle1)
    plt.title('Robot position in 2D',
              fontsize=14, fontweight='bold')
    plt.grid()
    plt.legend(loc="upper left")
    plt.show()


def plot_covariance(bag_file):
    b = bagreader(bag_file)
    cov = b.message_by_topic('/pose')
    covdf = pd.read_csv(cov)


if __name__ == '__main__':
    plot_trajectory('10-06-Bags/corredores.bag')
