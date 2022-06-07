#!/usr/bin/env python3
"""Subscriber module."""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters
import mapping
import numpy as np
import plots
import math

map = mapping.Map([-20, 20], [-20, 20], 0.1, 0.5)


def callback(scan, pose_msg):
    """Log listened data."""
    arr = np.linspace(scan.angle_min, scan.angle_max, 725)
    map.calculate_map(
        scan.ranges[1:725],
        arr,
        pose_msg.pose.pose.position.x,
        pose_msg.pose.pose.position.y,
        2 * math.atan2(
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w))
    print("Hello")
    print(pose_msg.pose.pose.position.x)
    print(pose_msg.pose.pose)


def listener():
    """Initialize a listener node to subscribe to a topic."""
    # The anonymous=True flag means that rospy will choose a unique
    # name for the 'listener' node so that multiple listeners can
    # run simultaneously.back function is::
    rospy.init_node('listener', anonymous=True)

    scan_sub = message_filters.Subscriber('scan', LaserScan)
    pose_sub = message_filters.Subscriber('pose', Odometry)

    rate = rospy.Rate(10)  # 10 Hz

    callback()
    rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    # rospy.init_node('listener', anonymous=True)
    # scan_sub = message_filters.Subscriber('scan', LaserScan)
    # pose_sub = message_filters.Subscriber('pose', Odometry)

    # ts = message_filters.ApproximateTimeSynchronizer([scan_sub, pose_sub], 10, 0.1)
    # ts.registerCallback(callback)

    # # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    listener()

    occupancy_map = map.return_map()
    plots.plot_map(occupancy_map, 0.1, [-10, 10], [-10, 10])
