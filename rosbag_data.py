#! /usr/bin/env python3
import numpy as np
import rosbag
import utils
import rosbag_mapping
import plots
import time
import rospy
import math


class Rosbag_handler():
    def __init__(self):

        # Robot pose parameters
        self.position_x = 0
        self.position_y = 0
        self.initial_x = 0
        self.initial_y = 0
        self.orientation = [0, 0, 0, 0]
        self.cov = 0
        self.pose_time = None

        # LRF parameters
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.time_increment = None
        self.range_min = None
        self.range_max = None
        self.ranges = 0
        self.scan_time = None
        self.angles = None

    def poseCallback(self, msg_pose):

        self.pose_time = msg_pose.header.stamp.secs
        self.position_x = msg_pose.pose.pose.position.x
        self.position_y = msg_pose.pose.pose.position.y
        self.orientation = [msg_pose.pose.pose.orientation.x, msg_pose.pose.pose.orientation.y,
                            msg_pose.pose.pose.orientation.z, msg_pose.pose.pose.orientation.w]
        self.cov = msg_pose.pose.covariance

    def scanCallback(self, msg_scan):

        self.scan_time = msg_scan.header.stamp.secs
        self.angle_min = msg_scan.angle_min
        self.angle_max = msg_scan.angle_max
        self.angle_increment = msg_scan.angle_increment
        self.time_increment = msg_scan.time_increment
        self.range_min = msg_scan.range_min
        self.range_max = msg_scan.range_max
        self.ranges = msg_scan.ranges
        self.angles = np.linspace(
            self.angle_min, self.angle_max, len(self.ranges))

    def convert_z(self, z):
        distances = self.ranges[0:]
        self.ranges = np.where(np.isnan(distances), 0, distances)
        return self.ranges

    def run_algorithm(self, i):
        # save initial positions of robot as (0, 0)
        if i == 1:
            self.initial_x = self.position_x
            self.initial_y = self.position_y

        _, __, yaw = utils.euler_from_quaternion(self.orientation)
        # returns data for mapping algorithm
        return self.ranges, self.angles, self.position_x-self.initial_x, self.position_y-self.initial_y, yaw, self.range_max

    def check_timestamps(self):

        difference = abs(self.pose_time - self.scan_time)
        print(difference)
        if difference <= 2:
            return True
        else:
            return False


if __name__ == '__main__':
    # desired limits and resolution of the map
    P_prior = 0.5
    xlim = [-20, 20]
    ylim = [-20, 20]
    resolution = 0.1
    # initialize 2-D occupancy grid map
    Map = rosbag_mapping.Map(xlim, ylim, resolution, P_prior)
    # read bag
    bag = rosbag.Bag('2022-05-31-15-10-35.bag')
    handler = Rosbag_handler()
    i = 0
    Scan = False
    Pose = False
    for topic, msg, t in bag.read_messages():
        if topic == '/pose':
            handler.poseCallback(msg)
            Scan = True
        elif topic == '/scan':
            handler.scanCallback(msg)
            Pose = True
        i += 1
        if (not(Scan) or not(Pose)):
            continue
        # if (handler.check_timestamps()):
        z, angles, x, y, yaw, z_max = handler.run_algorithm(i)
        handler.convert_z(z)
        occupancy_map = Map.calculate_map(z, angles, x, y, yaw, z_max)
        Pose = False
        Scan = False
    # plot results in plots.py
    plots.plot_map(occupancy_map, resolution, xlim, ylim)
    bag.close()
