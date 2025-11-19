#!/usr/bin/env python3
import math
import numpy as np
import rosbag2_py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from rclpy.serialization import serialize_message
import time
import cv2
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class CollectLidar(Node):
    _WIDTH = 513
    _HEIGHT = 513
    _M_PER_PIXEL = 0.05

    def __init__(self):
        super().__init__('collect_lidar')
        self.get_logger().info(f'{self.get_name()} created')

        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri="test_horizontal2",
            storage_id="mcap"
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

        # odom_topic = rosbag2_py.TopicMetadata(
        #     name="odom",
        #     type='nav_msgs/msg/Odometry',
        #     serialization_format='cdr'
        # )
        # self.writer.create_topic(odom_topic)

        horizontal_topic = rosbag2_py.TopicMetadata(
            name="horizontal",
            type='sensor_msgs/msg/LaserScan',
            serialization_format='cdr'
        )
        self.writer.create_topic(horizontal_topic)

        # vertical_topic = rosbag2_py.TopicMetadata(
        #     name="vertical",
        #     type='sensor_msgs/msg/LaserScan',
        #     serialization_format='cdr'
        # )
        # self.writer.create_topic(vertical_topic)

        # self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        # self.create_subscription(LaserScan, "/horizontal_scan", self._scan_callback_horizontal, 1)
        # self.create_subscription(LaserScan, "/vertical_scan", self._scan_callback_vertical, 1)

        self.create_subscription(LaserScan, "scan", self._scan_callback_horizontal, 1)

    def _scan_callback_horizontal(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        self.get_logger().info(f"lidar ({angle_min},{angle_max},{angle_increment},{len(ranges)})")
        
        self.writer.write("horizontal", serialize_message(msg), int(time.time()))

    def _scan_callback_vertical(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        self.writer.write("vertical", serialize_message(msg), int(time.time()))

    def _odom_callback(self, msg):
        pose = msg.pose.pose

        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitch, yaw = euler_from_quaternion(o)
        cur_t = yaw
        
        # self.get_logger().info(f"at ({cur_x},{cur_y},{cur_t})")
        serialized = serialize_message(msg)
        self.writer.write("odom", serialized, int(time.time()))

def main(args=None):
    rclpy.init(args=args)
    node = CollectLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

