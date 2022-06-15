#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
import node_mapper
import numpy as np
import plots
import math

# Define map parameters
width = 4000
height = 4000
resolution = 0.05

# Probability for unknown cells
P_prior = 0.5


class mappingNode(object):
    """Class for subscribing to topics and publishing calculated map."""
    # Instance mapping algorithm
    mapper = node_mapper.Map(width, height, resolution, P_prior)

    def __init__(self):

        # Robot pose parameters
        self.x = 0
        self.y = 0
        self.orientation = [0, 0, 0, 0]
        self.yaw = 0
        # Laser scan parameters
        self.ranges = []
        self.angles = []
        self.z_max = None
        self.z_min = None

        # Map parameters
        self.width = width
        self.height = height
        self.resolution = resolution

        # Subscribe to scan and amcl_pose messages
        rospy.Subscriber('scan', LaserScan, self.callback_scan)
        rospy.Subscriber(
            'amcl_pose', PoseWithCovarianceStamped, self.callback_pose)

        # Map message initial information
        self.grid_map = OccupancyGrid()
        self.grid_map.header.frame_id = "map"
        self.grid_map.info.resolution = self.resolution
        self.grid_map.info.width = self.width
        self.grid_map.info.height = self.height
        self.grid_map.info.origin.position.x = -100
        self.grid_map.info.origin.position.y = -100
        self.grid_map.info.origin.position.z = 0
        self.grid_map.info.origin.orientation.x = 0
        self.grid_map.info.origin.orientation.y = 0
        self.grid_map.info.origin.orientation.z = 0
        self.grid_map.info.origin.orientation.w = 0
        self.grid_map.data = []

        # Publish topic /my_map with the calculated map
        self.map_publisher = rospy.Publisher(
            '/my_map', OccupancyGrid, queue_size = 1)
        # Publish topic /my_map_metadata
        self.map_data_publisher = rospy.Publisher('/my_map_metadata', MapMetaData, queue_size = 1)
        # Publish initial map
        rospy.loginfo("Publishing initial map !")
        self.map_publisher.publish(self.grid_map)
        self.map_data_publisher.publish(self.grid_map.info)

        # OGM Algorithm
        self.occupancy_map = self.mapper.calculate_map(self.ranges,
                                                       self.angles,
                                                       self.x,
                                                       self.y,
                                                       self.yaw,
                                                       self.z_max,
                                                       self.z_min)
        self.probability_map = []
        self.threshold = 0.5

    def callback_pose(self, pose_msg):
        """Log listened pose data."""

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
        """Log listened laser data."""

        self.ranges = scan_msg.ranges
        self.ranges = np.where(np.isnan(self.ranges), 0, self.ranges)

        self.angles = np.linspace(scan_msg.angle_min,
                                  scan_msg.angle_max,
                                  len(self.ranges))
        self.z_max = scan_msg.range_max
        self.z_min = scan_msg.range_min

        self.occupancy_map = self.mapper.calculate_map(self.ranges,
                                                       self.angles,
                                                       self.x,
                                                       self.y,
                                                       self.yaw,
                                                       self.z_max,
                                                       self.z_min)

    def publish_map(self):
        """ Convert the map data to a list and its respective occupancy probabilities 
        and publish the topic to ROS.
        The map data is a list and the Occupancy probabilities are in the range [0, 100]. 
        Unknown is -1."""

        # restore probability from log-odds
        self.probability_map = 1 - (1 / (1 + np.exp(self.occupancy_map)))

        # define cell thresholds and apply occupancy probabilities
        unknown = (np.where(self.probability_map == 0.5))
        self.probability_map[unknown] = -1

        occupied = (np.where(self.probability_map > 0.5))
        self.probability_map[occupied] = 100

        free = (np.where(self.probability_map <
                self.threshold) and np.where(self.probability_map >= 0))
        self.probability_map[free] = 0

        # convert map to a list of occupancy values and publishes to ROS
        temp = np.reshape(self.probability_map, (1, self.width*self.height))
        self.grid_map.data = temp.tolist()[0]
        self.grid_map.data = np.int8(np.round_(self.grid_map.data))
        rospy.loginfo("Publishing updated map ! ")
        self.map_publisher.publish(self.grid_map)
        self.map_data_publisher.publish(self.grid_map.info)


def main():
    rospy.init_node('mapping_node', anonymous=True)
    my_node = mappingNode()

    # ROS node rate to get messages
    rate = rospy.Rate(0.1)  # 10 Hz

    while not rospy.is_shutdown():
        my_node.occupancy_map = my_node.mapper.return_map()
        rate.sleep()
        my_node.publish_map()


if __name__ == '__main__':
    main()
