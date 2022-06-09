#! /usr/bin/env python3
import matplotlib.pyplot as plt
import cv2 as cv2
from bagpy import bagreader
import pandas as pd
import mapping


def plot_map(occupancy_map, resolution, xlim, ylim):
    '''Plot a simple figure to represent the occupancy map '''

    plt.figure(0)
    plt.imshow(mapping.restore_p(occupancy_map), 'Blues')

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
    plt.show()


if __name__ == '__main__':

    # run to get robot's trajectory (x, y)

    b = bagreader('2022-05-31-15-10-35.bag')
    pose = b.message_by_topic('/pose')
    posedf = pd.read_csv(pose)

    posedf.plot(x='pose.pose.position.x', y='pose.pose.position.y',
                xlabel='x [m]', ylabel='y [m]')
    plt.title('Position of the robot',
              fontsize=14, fontweight='bold')
    plt.show()
