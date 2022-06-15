#!/usr/bin/env python3
"""Subscriber module."""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import mapping
import numpy as np
import plots
import math


class MySubscriber(object):
    """Class for subscribing to topics."""

    map = mapping.Map([-20, 20], [-20, 20], 0.1, 0.5)

    def __init__(self):
        """Initialize new MySubscriber object."""
        self.x = 0
        self.y = 0
        self.orientation = [0, 0, 0, 0]
        self.yaw = 0
        self.flag = 0
        self.initial_x = 0
        self.initial_y = 0

        self.ranges = []
        self.angles = []
        self.z_max = 0
        self.z_min = 0

        self.occupancy_map = self.map.calculate_map(self.ranges,
                                                    self.angles,
                                                    self.x,
                                                    self.y,
                                                    self.yaw,
                                                    self.z_max, self.z_min)

        rospy.Subscriber('scan', LaserScan, self.callback_scan)
        rospy.Subscriber(
            'amcl_pose', PoseWithCovarianceStamped, self.callback_pose)
        #rospy.Subscriber('pose', Odometry, self.callback_pose)


    def callback_pose(self, pose_msg):
        """Log listened data."""
        print('x = ' + str(self.x) + ', ' +
              'y = ' + str(self.y) + ', ' +
              'yaw = ' + str(self.yaw))
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        self.orientation = [pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,
                            pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]

        self.yaw = 2 * math.atan2(self.orientation[2],
                                  self.orientation[3])

    def callback_scan(self, scan_msg):
        """Log listened data."""
        self.ranges = scan_msg.ranges
        self.ranges = np.where(np.isnan(self.ranges), 0, self.ranges)

        self.angles = np.linspace(scan_msg.angle_min,
                                  scan_msg.angle_max,
                                  len(self.ranges))
        self.z_max = scan_msg.range_max
        self.z_min = scan_msg.range_min

        self.occupancy_map = self.map.calculate_map(self.ranges,
                                                    self.angles,
                                                    self.x,
                                                    self.y,
                                                    self.yaw,
                                                    self.z_max,
                                                    self.z_min)

def main():
    rospy.init_node('mapping_node', anonymous=True)
    my_node = MySubscriber()

    # ROS node rate to get messages
    #rate = rospy.Rate(0.1)  # 10 Hz

    while not rospy.is_shutdown():
        my_node.occupancy_map = my_node.map.return_map()
        #rate.sleep()
    plots.plot_map(my_node.occupancy_map, 0.1, [-20, 20], [-20, 20])


if __name__ == '__main__':
    main()
