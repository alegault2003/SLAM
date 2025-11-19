#!/usr/bin/env python3

"""
Pose Listener for slam_toolbox
Should suscribe to the pose topic from slam_toolbox and logs poses to a CSV file
Compatibles with YDLiDAR G4
"""
import rclpy
from rclpy.node import Node
from geometry_msg.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import csv
import math
from datetime import datetime
import os

class PoseListener(Node):
    """
    This is the ROS2 node that listens to slam_toolbox pose updates and logs them to CSV
    """
    def __init__(self, output_file='poses_log.txt'):
        super().__init__('pose_listener')
        self.output_file = output_file
        self.pose_count = 0

        # To create a CSV file
        self.setup_output_file()

        # Subscribe to slam_toolbox pose topic
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

        # Alternative using odometry
        # only if sensor is publishing to /odom
        self.odom_subscription = self.create_subscription(
           Odometry,
           '/odom',
           self.odom_callback,
           10
        )
        self.get_logger().info(f'Pose listener started. Logging to {self.output_file}')
        self.get_logger().info('Waiting for pose messages...')

    def setup_output_file(self):
        """Create a csv file with header"""
        if os.path.exists(self.output_file):
            backup_name = f"{self.output_file}.backup_{datetime.now().strftime('%Y%m%d%H%M%S')}"
            self.get_logger().warn(f'Output file exists! Creating backup: {backup_name}')
            os.rename(self.output_file, backup_name)

        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp','x', 'y', 'theta'])

        self.get_logger().info(f'Created output file: {self.output_file}')

    def pose_callback(self, msg):
        """
        Callback for PoseWithCovarianceStamped messages from slam_toolbox
        """
        # Extract timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nsecs * 1e-9

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation (convert quaternion to yaw angle)
        orientation_q = msg.pose.pose.orientation
        theta = self.quaternion_to_yaw(orientation_q)

    def odom_callback(self, msg):
        """
        Alternative callback for odometry messages
        """
        timestamp = msg.header.stamp.sec + msg.header.stamp.nsecs * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        theta = self.quaternion_to_yaw(orientation_q)

        self.write_pose(timestamp, x, y, theta)

    def quaternion_to_yaw(self, q):
        """
        Convert a quaternion to yae angle (theta)
        Args:
            q: geometry_msgs.msg.Quaternion

        Returns:
            float: yaw angle in radians
        """
        siny_cosp = 2*(q.W*q.z+q.x*q.y)
        cosy_cosp = 1-2*(q.y*q.y+q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def write_pose(self, timestamp, x, y, theta):
        """Write pose data to csv file"""
        with open(self.output_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp, x, y, theta])
        self.pose_count += 1

        if self.pose_count % 50 == 0:
            self.get_logger().info(
                f'Logged {self.pose_count} poses | '
                f'Latest: x={x:.3f}, y={y:.3f}, theta={theta:.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    pose_listener = PoseListener(output_file='poses_log.txt')
    try:
        rclpy.spin(pose_listener)
    except KeyboardInterrupt:
        pose_listener.get_logger().info(f'\nShutting down. Total poses logged: {pose_listener.pose_count}')
    finally:
        pose_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

