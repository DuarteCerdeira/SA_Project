#!/usr/bin/env python3
"""Subscriber module."""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import mapping
import numpy as np
import plots
import math



PI = 3.1415

class MySubscriber(object):
    """Class for subscribing to topics."""

    map = mapping.Map([-20, 20], [-20, 20], 0.1, 0.5)

    def __init__(self):
        """Initialize new MySubscriber object."""
        self.x = 0
        self.y = 0
        self.theta = 0

        self.ranges = []
        self.angles = []
        self.z_max = 0

        self.occupancy_map = self.map.calculate_map(self.ranges,
                                                    self.angles,
                                                    self.x,
                                                    self.y,
                                                    self.theta,
                                                    self.z_max)

        rospy.Subscriber('scan', LaserScan, self.callback_scan)
        rospy.Subscriber('pose', Odometry, self.callback_pose)

    def callback_pose(self, pose_msg):
        """Log listened data."""
        print('x = ' + str(self.x) + ', ' +
              'y = ' + str(self.y) + ', ' +
              'theta = ' + str(self.theta))
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y
        xq = pose_msg.pose.pose.orientation.x
        yq = pose_msg.pose.pose.orientation.y
        zq = pose_msg.pose.pose.orientation.z
        wq = pose_msg.pose.pose.orientation.w
        # yaw_z = tf.transformations.euler_from_quaternion(msg_pose.pose.orientation, axes='z')
        t3 = +2.0 * (wq * zq + xq * yq)
        t4 = +1.0 - 2.0 * (yq * yq + zq * zq)
        yaw_z = math.atan2(t3, t4) + PI/2
        self.theta = yaw_z
        # self.theta = 2 * math.atan2(pose_msg.pose.pose.orientation.z,
        #                            pose_msg.pose.pose.orientation.w)

    def callback_scan(self, scan_msg):
        """Log listened data."""
        self.ranges = scan_msg.ranges[1:None]
        self.ranges = np.where(np.isnan(self.ranges), 0, self.ranges)

        self.angles = np.linspace(scan_msg.angle_min,
                                  scan_msg.angle_max,
                                  len(self.ranges))

        self.z_max = scan_msg.range_max

        self.occupancy_map = self.map.calculate_map(self.ranges,
                                                    self.angles,
                                                    self.x,
                                                    self.y,
                                                    self.theta,
                                                    self.z_max)

    def loop(self):
        """Start main loop."""
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('subscriber_node', anonymous=True)
    my_sub = MySubscriber()
    my_sub.loop()
    my_sub.occupancy_map = my_sub.map.return_map()
    plots.plot_map(my_sub.occupancy_map, 0.1, [-20, 20], [-20, 20])
