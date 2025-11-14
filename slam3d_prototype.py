#!/usr/bin/env python3
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
import json


# === HELPER LIBRARIES (USE IF NEEDED) ====
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from pathlib import Path
from tqdm import tqdm

@dataclass
class Pose2D:
    x: float
    y: float
    theta: float
    timestamp: float


@dataclass
class LiDARScan:
    ranges: np.ndarray
    angles: np.ndarray
    timestamp: float


@dataclass
class Point3D:
    x: float
    y: float
    z: float
    intensity: float = 1.0


class VerticalLiDARTransformer:
    """
    Transforms vertical LiDAR scans into 3D points using 2D SLAM poses
    Based on the formula from Source 5 (CMU paper) see resources on Discord
    """

    def __init__(self, vertical_offset: float = 0.0):
        """
        Args:
            vertical_offset: Height offset of vertical LiDAR from robot base (meters)
        """
        self.vertical_offset = vertical_offset

    def transform_scan_to_3d(self,
                             scan: LiDARScan,
                             pose: Pose2D) -> List[Point3D]:
        """
        Transform a vertical LiDAR scan to 3D points using robot pose

        Formula: For each point (r, alpha) in vertical scan:
            x_3d = x_robot + r * cos(alpha) * cos(theta_robot)
            y_3d = y_robot + r * cos(alpha) * sin(theta_robot)
            z_3d = z_offset + r * sin(alpha)

        where:
            r = range measurement
            alpha = vertical angle of the scan ray
            (x_robot, y_robot, θ_robot) = 2D pose from SLAM
        """
        points_3d = []

        for r, alpha in zip(scan.ranges, scan.angles):
            if r <= 0 or np.isnan(r) or np.isinf(r):
                continue

            horizontal_dist = r * np.cos(alpha)

            x_3d = pose.x + horizontal_dist * np.cos(pose.theta)
            y_3d = pose.y + horizontal_dist * np.sin(pose.theta)
            z_3d = self.vertical_offset + r * np.sin(alpha)

            points_3d.append(Point3D(x_3d, y_3d, z_3d))

        return points_3d


class DualLiDARSLAM:
    """
    Main SLAM system coordinating 2D SLAM and 3D reconstruction
    """

    def __init__(self,
                 horizontal_lidar_topic: str = "/scan_horizontal",
                 vertical_lidar_topic: str = "/scan_vertical",
                 vertical_offset: float = 0.15):
        """
        Args:
            horizontal_lidar_topic: ROS topic for horizontal LiDAR
            vertical_lidar_topic: ROS topic for vertical LiDAR
            vertical_offset: Height of vertical LiDAR above ground (meters)
        """
        self.horizontal_topic = horizontal_lidar_topic
        self.vertical_topic = vertical_lidar_topic

        self.transformer = VerticalLiDARTransformer(vertical_offset)

        self.point_cloud_3d: List[Point3D] = []

        self.current_pose: Optional[Pose2D] = None

        self.pose_history: List[Pose2D] = []

    def add_scan_with_pose(self, scan:LiDARScan, pose: Pose2D):
        new_points = self.transformer.transform_scan_to_3d(scan, pose)
        self.point_cloud_3d.extend(new_points)

    def update_pose_from_slam(self, x: float, y: float, theta: float, timestamp: float):
        """
        Update current robot pose from slam_toolbox
        This would be called from ROS2 pose callback
        """
        self.current_pose = Pose2D(x, y, theta, timestamp)
        self.pose_history.append(self.current_pose)

    def process_vertical_scan(self, scan: LiDARScan) -> List[Point3D]:
        """
        Process a vertical LiDAR scan and add to 3D point cloud
        """
        if self.current_pose is None:
            print("Warning: No pose available yet, skipping scan")
            return []

        new_points = self.transformer.transform_scan_to_3d(scan, self.current_pose)

        self.point_cloud_3d.extend(new_points)

        return new_points

    def get_point_cloud(self) -> np.ndarray:
        if not self.point_cloud_3d:
            return np.array([]).reshape(0, 3)

        return np.array([[p.x, p.y, p.z] for p in self.point_cloud_3d])

    def clear_point_cloud(self):
        self.point_cloud_3d.clear()

    def export_point_cloud(self, filename: str, format: str = "xyz"):
        points = self.get_point_cloud()

        if format == "xyz":
            np.savetxt(filename, points, fmt='%.6f')
        elif format == "ply":
            self._export_ply(filename, points)
        else:
            raise ValueError(f"Unsupported format: {format}")

    def _export_ply(self, filename: str, points: np.ndarray):
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")

            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")

MSG_TYP = """
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32
        uint32 nanosec
    string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
"""
typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(get_types_from_msg(MSG_TYP, 'sensor_msgs/msg/LaserScan'))

def load_poses(pose_file:str) -> pd.DataFrame:
    """
    Loads the poses_log.txt into a pd.DataFrame
    """
    print(f"Loading poses from {pose_file}...")
    try:
        df = pd.read_csv(pose_file)
    except FileNotFoundError:
        print(f"ERROR: Pose file not found at {pose_file}")
        print("Please run the pose_listener.py script first.")
        exit(1)

    df = df.sort_values(by='timestamp')
    df = df.set_index('timestamp')
    print(f"Loader {len(df)} poses")
    return df

def extract_vertical_scans(bag_file: str, vertical_topic: str)->List[LiDARScan]:
    """
    Reads a .bag and extracts all LaserScan messages
    from the specified vertical topic
    """
    print(f"Extracting vertical scans from {bag_file} on topic {vertical_topic}...")
    scans = []

    with AnyReader([Path(bag_file)]) as reader:
        if vertical_topic not in reader.topics:
            print(f"ERROR: Vertical topic {vertical_topic} not found in {bag_file}")
            print(f"Available topics: {list(reader.topics.keys())}")
            exit(1)

        connections = [x for x in reader.connections if x.topic == vertical_topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ranges = np.array(msg.ranges, dtype=np.float32)
            print(msg.header.frame_id)

        if hasattr(msg, 'angle_min') and hasattr(msg, 'angle_increment') and len(ranges) > 0:
            num = len(ranges)
            angles = msg.angle_min + np.arange(num, dtype=np.float32) * msg.angle_increment
            angles = angles.astype(np.float32)
        else:
            angles = np.array([], dtype=np.float32)

        scans.append(LiDARScan(ranges, angles, timestamp))

    print(f"Extracted {len(scans)} vertical scans")
    return scans

def find_close_pose(timestamp: float, pose_df:pd.DataFrame, max_dt:float=0.1) -> Optional[Pose2D]:
    try:

        if pose_df is None or len(pose_df) == 0:
            return None

        idx = pose_df.index.searchsorted(timestamp)
        candidates = []
        if idx < len(pose_df):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx-1)

        best_idx = None
        best_dt = None

        for i in candidates:
            dt = abs(pose_df.index[i]-timestamp)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_idx = i

        if best_idx is None or best_dt is None or best_dt > max_dt:
            return None

        pose_series = pose_df.iloc[best_idx]

        return Pose2D(
            x=float(pose_series['x']),
            y=float(pose_series['y']),
            theta=float(pose_series['theta']),
            timestamp=float(pose_series.name)
        )
    except KeyError:
        return None


class OctomapIntegration:
    def __init__(self, resolution: float = 0.05):
        self.resolution = resolution
        self.occupied_voxels = set()

    def insert_point_cloud(self, points: np.ndarray, sensor_origin: Tuple[float, float, float]):
        for point in points:
            voxel = self._point_to_voxel(point)
            self.occupied_voxels.add(voxel)
            # Here I would implement Bresenham 3D for Ray-Tracing (didn't do it yet still thinking of how I would do it)

    def _point_to_voxel(self, point: np.ndarray) -> Tuple[int, int, int]:
        return (
            int(np.floor(point[0] / self.resolution)),
            int(np.floor(point[1] / self.resolution)),
            int(np.floor(point[2] / self.resolution))
        )

    def get_occupied_voxels(self) -> List[Tuple[int, int, int]]:
        return list(self.occupied_voxels)


def main():
    """
    Offline Processing
    """
    BAG_FILE = "SET BAG FILE HERE"
    POSE_FILE = "THE FILE WE GET FROM POSE LISTENER"
    VERTICAL_SCAN_TOPIC = "OUR VERTICAL LIDAR TOPIC"
    VERTICAL_OFFSET_METERS = 0.15 # Default is set to 15 cm, we will set the height of our LiDAR here
    OUTPUT_FILE = "NAME OF OUR OUTPUT MAP (.ply or .xyz)"

    pose_df = load_poses(POSE_FILE)

    scan_data = extract_vertical_scans(BAG_FILE, VERTICAL_SCAN_TOPIC)

    if not scan_data:
        print("No scan data found. Exiting.")
        return
    print("Aligning poses and processing scans to build 3D map...")
    slam = DualLiDARSLAM(vertical_offset=VERTICAL_OFFSET_METERS)
    scans_processed = 0
    scans_skipped = 0

    for scan in tqdm(scan_data, desc="Processing Scans"):
        pose = find_close_pose(scan.timestamp, pose_df)

        if pose:
            slam.add_scan_with_pose(scan, pose)
            scans_processed+=1
        else:
            scans_skipped+=1


    print("...Processing complete.")
    print(f"Total scans processed: {scans_processed}")
    print(f"Total scans skipped: {scans_skipped}")

    print(f"\nExporting point cloud to {OUTPUT_FILE}...")
    slam.export_point_cloud(OUTPUT_FILE, format="ply")

    total_points = len(slam.point_cloud_3d)
    if total_points > 0:
        print(f"Successfully exported {total_points} points.")
        print(f"You can now open {OUTPUT_FILE}.")
    else:
        print("No points were generated. The map is empty")


if __name__ == "__main__":
    main()