#! /usr/bin/env python3
import matplotlib.pyplot as plt
from mapping import restore_p
import cv2 as cv2


def plot_map(occupancy_map, resolution, xlim, ylim):
    '''Plot a simple figure to represent the occupancy map '''

    plt.figure(0)
    plt.imshow(restore_p(occupancy_map), 'Blues')

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


def read_map_files(file):
    image = cv2.imread(file, -1)
    plt.figure(1)
    plt.imshow(image, 'gray')
    plt.xlim([1700, 2100])
    plt.ylim([1900, 2400])
    plt.title('Gmapping reference occupancy map',
              fontsize=14, fontweight='bold')
    plt.colorbar()
    plt.show()
