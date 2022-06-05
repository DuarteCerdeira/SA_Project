#!/usr/bin/env python
"""Subscriber module."""

import rospy
from sensor_msgs.msg._LaserScan import LaserScan


def callback(data):
    """Log listened data."""
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print(data.ranges[1:725])


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
