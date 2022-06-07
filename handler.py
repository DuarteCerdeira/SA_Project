#!/usr/bin/env python3
"""Subscriber module."""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import mapping
import numpy as np
import plots
import math

map = mapping.Map([-20, 20], [-20, 20], 0.1, 0.5)


def callback_pose(pose_msg):
    """Log listened data."""
    global x
    global y
    global theta
    x = pose_msg.pose.pose.position.X
    y = pose_msg.pose.pose.position.y
    theta = 2 * math.atan2(
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w)


def callback_scan(scan_msg):
    global ranges
    global angles
    global z_max
    ranges = scan_msg.ranges[1:None]
    ranges = np.where(np.isnan(ranges), 0, ranges)

    angles = np.linspace(scan_msg.angle_min,
                         scan_msg.angle_max, len(ranges))

    z_max = scan_msg.range_max


def listener_pose():
    """Initialize a listener node to subscribe to a topic."""
    # The anonymous=True flag means that rospy will choose a unique
    # name for the 'listener' node so that multiple listeners can
    # run simultaneously.back function is::
    rospy.init_node('listener_pose', anonymous=True)
    pose_sub = rospy.Subscriber('pose', Odometry, callback_scan)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def listener_pose():
    """Initialize a listener node to subscribe to a topic."""
    # The anonymous=True flag means that rospy will choose a unique
    # name for the 'listener' node so that multiple listeners can
    # run simultaneously.back function is::
    rospy.init_node('listener_scan', anonymous=True)
    scan_sub = rospy.Subscriber('scan', LaserScan, callback_pose)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        listener_pose()
        listener_scan()
        occupancy_map = map.calculate_map(ranges, angles, x, y, theta, z_max)
        r.sleep()

    occupancy_map = map.return_map()
    plots.plot_map(occupancy_map, 0.1, [-20, 20], [-20, 20])
