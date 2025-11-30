#!/usr/bin/env python3

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
import time

robot_height = 0.3
laser_height = 0.08
laser_radius = 0.1

# origin2pose = pose
# pose2robot_base = pose + robot_height/2 (z)
# robot_base2hor_lidar = pose2robot_base + robot_height/2 + laser_heigth/2 (z-axis)
# hor_lidar2ver_lidar = robot_base2hor_lidar + laser_heigth/2 + laser_radius (z-axis) + 90-deg rotation (x or y axis)

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

def sync_streams(odom_list, hor_list, ver_list, imu_list, max_dt_ns=5e7):
    """
    Takes 3 lists of (timestamp, msg)
    Returns a list of tuples (odom, hor_scan, ver_scan)
    """
    import bisect

    # Make arrays of timestamps
    hor_times = [t for t, _ in hor_list]
    ver_times = [t for t, _ in ver_list]
    odom_times = [t for t, _ in odom_list]

    synced = []

    for imu_t, imu_msg in imu_list:
        # ---- Match horizontal scan ----
        idx = bisect.bisect_left(hor_times, imu_t)
        best_h = None
        best_dt = max_dt_ns

        for j in (idx, idx-1):
            if 0 <= j < len(hor_list):
                t, m = hor_list[j]
                dt = abs(t - imu_t)
                if dt < best_dt:
                    best_dt = dt
                    best_h = (t, m)

        if best_h is None:
            continue  # no matching horizontal scan

        # ---- Match vertical scan ----
        idx = bisect.bisect_left(ver_times, imu_t)
        best_v = None
        best_dt2 = max_dt_ns

        for j in (idx, idx-1):
            if 0 <= j < len(ver_list):
                t, m = ver_list[j]
                dt = abs(t - imu_t)
                if dt < best_dt2:
                    best_dt2 = dt
                    best_v = (t, m)

        if best_v is None:
            continue  # no matching vertical scan

        # ---- Match odom ----
        idx = bisect.bisect_left(odom_times, imu_t)
        best_o = None
        best_dt3 = max_dt_ns

        for j in (idx, idx-1):
            if 0 <= j < len(odom_list):
                t, m = odom_list[j]
                dt = abs(t - imu_t)
                if dt < best_dt3:
                    best_dt3 = dt
                    best_o = (t, m)

        if best_o is None:
            continue  # no matching vertical scan

        # Final matched triplet
        synced.append((imu_t, imu_msg, best_h[1], best_v[1], best_o[1]))

    return synced


def read_bag(bag_path):
    all_pointclouds = []
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="mcap"
    )
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    odom_info = []
    hor_lidar_info = []
    ver_lidar_info = []
    imu_info = []
    remove = False

    while reader.has_next():
        topic, data, recv_ts = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg_deserialized = deserialize_message(data, msg_type)
        
        if isinstance(msg_deserialized, Imu) and topic == "imu":
            imu_info.append((recv_ts, msg_deserialized))
        
        if isinstance(msg_deserialized, Odometry) and topic == "odom":
            odom_info.append((recv_ts, msg_deserialized))
            

        if isinstance(msg_deserialized, LaserScan):
            if topic == "horizontal":
                hor_lidar_info.append((recv_ts, msg_deserialized))
            elif topic == "vertical":
                ver_lidar_info.append((recv_ts, msg_deserialized))

    synced_msgs = sync_streams(odom_info, hor_lidar_info, ver_lidar_info, imu_info)
    synced_imu = []
    synced_odom = []
  

    for imu_t, imu_msg, hor_scan, ver_scan, odom in synced_msgs:
        synced_imu.append((imu_t, imu_msg))
        synced_odom.append((imu_t, odom))
        frame_cloud = []
        pose = odom.pose.pose

        cur_x = pose.position.x
        cur_y = pose.position.y
        cur_z = pose.position.z
        robot_coor = [cur_x, cur_y, cur_z]

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        cur_t = yaw
        
        hor_angle_min = hor_scan.angle_min
        hor_angle_max = hor_scan.angle_max
        hor_angle_increment = hor_scan.angle_increment
        hor_ranges = hor_scan.ranges     


        # hor_rotation = np.array([
        #     [np.cos(yaw), -np.sin(yaw), 0],
        #     [np.sin(yaw), np.cos(yaw), 0],
        #     [0, 0, 1]
        # ])
        # hor_translation = np.array([
        #     [robot_coor[0]],
        #     [robot_coor[1]],
        #     [robot_coor[2] + robot_height + laser_height/2],
        #     [1]
        # ])

        # # trans_hor2odom = np.concatenate(np.append(hor_rotation, rotation_zeros, axis=0), np.append(hor_translation, translation_one, axis=0), axis=-1)
        # trans_hor2odom = np.eye(4)
        # trans_hor2odom[0:3, 0:3] = hor_rotation
        # trans_hor2odom[0:3, 3] = hor_translation[0:3,0]

        trans_hor2base = np.eye(4)
        trans_hor2base[2, 3] = robot_height + laser_height/2

        # vertical_rotation = np.array([
        #     [1, 0, 0],
        #     [0, 0, -1],
        #     [0, 1, 0]
        # ])
        # ver_rotation = vertical_rotation
        # ver_translation = np.array([
        #     [hor_translation[0,0]],
        #     [hor_translation[1,0]],
        #     [hor_translation[2,0] + laser_height/2 + laser_radius]
        # ])

        # # trans_ver2odom = np.concatenate(np.append(ver_rotation, rotation_zeros, axis=0), np.append(ver_translation, translation_one, axis=0), axis=-1)
        # trans_ver2odom = np.eye(4)
        # trans_ver2odom[0:3, 0:3] = hor_rotation
        # trans_ver2odom[0:3, 3] = ver_translation[0:3,0]

        trans_ver2base = np.eye(4)
        trans_hor2base[2, 3] = robot_height + laser_height + laser_radius

        angle = hor_angle_min
        hor_ranges_xy = []
        for r in hor_ranges:
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            point = np.array([
                [x],
                [y],
                [0],
                [1]
            ])
            point_transformed = trans_hor2base @ point
            frame_cloud.append(point_transformed[:3])
          
            hor_ranges_xy.append(point_transformed)
            angle = angle + hor_angle_increment

        ver_angle_min = ver_scan.angle_min
        ver_angle_max = ver_scan.angle_max
        ver_angle_increment = ver_scan.angle_increment
        ver_ranges = ver_scan.ranges

        angle = ver_angle_min
        ver_ranges_xz = []
        for r in ver_ranges:
            x = r * np.cos(angle)
            z = r * np.sin(angle)

            point = np.array([
                [x],
                [0],
                [z],
                [1]
            ])
            # point_transformed = point
            point_transformed = trans_ver2base @ point
            frame_cloud.append(point_transformed[:3])
            ver_ranges_xz.append(point_transformed)
            angle = angle + ver_angle_increment
        all_pointclouds.append(np.array(frame_cloud))
    return all_pointclouds, synced_imu, synced_odom

def numpy_to_pointcloud2(points, frame_id="base_link"):
    """
    points: Nx3 numpy array
    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    header = Header()
    header.stamp = rclpy.time.Time().to_msg()
    header.frame_id = frame_id

    pc2_msg = pc2.create_cloud(header, fields, points)
    return pc2_msg

class BagPointCloudPublisher(Node):
    
    def __init__(self, frame_clouds, imu, odom, topic_name="/points2", frame_id="base_link"):
        super().__init__("bag_pointcloud_publisher")
        self.pub = self.create_publisher(PointCloud2, '/points2', 1)
        self.imu_pub = self.create_publisher(Imu, "/imu", 1)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 1)
        self.frame_clouds = frame_clouds
        self.frame_id = frame_id
        self.imu = imu
        self.odom = odom
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.index = 0
         # --- Timestamp shifting setup ---
        self.first_bag_stamp = Time(
            seconds=self.imu[0][1].header.stamp.sec,
            nanoseconds=self.imu[0][1].header.stamp.nanosec,
        )
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        self.first_wall_time = Time(seconds=sec, nanoseconds=nsec)
        self.offset = self.first_wall_time - self.first_bag_stamp

    
    def timer_callback(self):
        if self.index >= len(self.frame_clouds):
            self.get_logger().info("Finished publishing all frames")
            return
        
        points = np.array(self.frame_clouds[self.index])
        points = points.reshape((-1, 3)).astype(np.float32)
        pc2_msg = numpy_to_pointcloud2(points.squeeze(), self.frame_id)
    
        imu_msg = self.imu[self.index][1]
        odom_msg = self.odom[self.index][1]
        
        original_stamp = Time(
            seconds=imu_msg.header.stamp.sec,
            nanoseconds=imu_msg.header.stamp.nanosec,
        )
        shifted_stamp = original_stamp + self.offset
        # 
        # t_footprint2link = TransformStamped()
        # t_footprint2link.header.stamp = shifted_stamp.to_msg()
        # t_footprint2link.header.frame_id = "base_footprint"
        # t_footprint2link.child_frame_id = self.frame_id
        
        # t_footprint2link.transform.translation.x = 0.0
        # t_footprint2link.transform.translation.y = 0.0
        # t_footprint2link.transform.translation.z = 0.0

        # t_footprint2link.transform.rotation.x = 0.0
        # t_footprint2link.transform.rotation.y = 0.0
        # t_footprint2link.transform.rotation.z = 0.0
        # t_footprint2link.transform.rotation.w = 1.0

        # self.tf_broadcaster.sendTransform(t_footprint2link)

        t_odom2footprint = TransformStamped()
        t_odom2footprint.header.stamp = shifted_stamp.to_msg()
        t_odom2footprint.header.frame_id = 'odom'
        t_odom2footprint.child_frame_id = self.frame_id
        
        t_odom2footprint.transform.translation.x = 0.0
        t_odom2footprint.transform.translation.y = 0.0
        t_odom2footprint.transform.translation.z = 0.0

        t_odom2footprint.transform.rotation.x = 0.0
        t_odom2footprint.transform.rotation.y = 0.0
        t_odom2footprint.transform.rotation.z = 0.0
        t_odom2footprint.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t_odom2footprint)

        pc2_msg.header.stamp = shifted_stamp.to_msg()
        pc2_msg.header.frame_id = self.frame_id
        imu_msg.header.stamp = shifted_stamp.to_msg()
        imu_msg.header.frame_id = self.frame_id
        odom_msg.header.stamp = shifted_stamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = self.frame_id
        self.pub.publish(pc2_msg)
        self.imu_pub.publish(imu_msg)
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published frame {self.index} with {points.shape[0]} points at {shifted_stamp}")
        self.index += 1


def main(args=None):

    rclpy.init(args=args)

    # Read rosbag and get frame clouds
    frame_clouds, imu, odom = read_bag("/media/agathe/External HD/sim_data_imu")

    node = BagPointCloudPublisher(frame_clouds, imu, odom)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
