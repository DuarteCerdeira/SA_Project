#!/usr/bin/env python
"""Subscriber module."""

import rospy
from sensor_msgs.msg._LaserScan import LaserScan
import mapping
import numpy as np

map = mapping.Map([-10, 10], [-10, 10], 0.1, 0.5)
occupancy_map = map.calculate_map(0, 0, 0, 0, 0)

def callback(data):
    """Log listened data."""
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print(data.ranges[1:725])
    arr = np.linspace(-2.356194, 2.09235, 725)
    occupancy_map = map.calculate_map(data.ranges[1:725], arr, 0, 0, 0)


def listener():
    """Initialize a listener node to subscribe to a topic."""
    # The anonymous=True flag means that rospy will choose a unique
    # name for the 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
    plots.plot_map(occupancy_map, 0.1, [-10, 10], [-10, 10])
