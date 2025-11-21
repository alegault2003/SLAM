#!/usr/bin/env python3
"""
=============================================================================
DUAL-LIDAR 3D MAPPING SYSTEM
=============================================================================

This code creates 3D maps using TWO LiDAR sensors:
  1. Horizontal LiDAR → used by slam_toolbox (separate program) for 2D SLAM
  2. Vertical LiDAR → processed by THIS code to add height information

WORKFLOW:
  Step 1: Run slam_toolbox + pose_listener.py → generates poses_log.txt
  Step 2: Record vertical LiDAR to bag file during same run
  Step 3: Run THIS script offline to combine them into 3D map

OUTPUT: A .ply or .xyz file containing a 3D point cloud
=============================================================================
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from pathlib import Path
from tqdm import tqdm


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class Pose2D:
    """Robot's position and orientation in 2D (from slam_toolbox)"""
    x: float  # X position in meters
    y: float  # Y position in meters
    theta: float  # Rotation angle in radians
    timestamp: float  # When this pose occurred (in seconds)


@dataclass
class LiDARScan:
    """One scan from the vertical LiDAR"""
    ranges: np.ndarray  # Distance measurements (in meters)
    angles: np.ndarray  # Angle for each measurement (in radians)
    timestamp: float  # When this scan occurred (in seconds)


@dataclass
class Point3D:
    """A single point in 3D space"""
    x: float
    y: float
    z: float
    intensity: float = 1.0


# =============================================================================
# COORDINATE TRANSFORMATION (The Core Math)
# =============================================================================

class VerticalLiDARTransformer:
    """
    Converts vertical LiDAR measurements into 3D world coordinates

    THE KEY IDEA:
    - Vertical LiDAR gives us (distance, vertical_angle) in robot's frame
    - We know robot's position (x, y, theta) from slam_toolbox
    - We combine them to get 3D points in world coordinates

    Formula from CMU paper:
        x_3d = x_robot + r·cos(α)·cos(θ_robot)
        y_3d = y_robot + r·cos(α)·sin(θ_robot)
        z_3d = z_offset + r·sin(α)

    where r = range, α = vertical angle, θ = robot orientation
    """

    def __init__(self, vertical_offset: float = 0.0):
        """
        Args:
            vertical_offset: Height of vertical LiDAR above ground (meters)
                           For example, if LiDAR is 15cm above floor, use 0.15
        """
        self.vertical_offset = vertical_offset

    def transform_scan_to_3d(self, scan: LiDARScan, pose: Pose2D) -> List[Point3D]:
        """
        Transform one vertical scan into 3D points

        Args:
            scan: Vertical LiDAR scan (ranges and angles)
            pose: Where the robot was when it took this scan

        Returns:
            List of 3D points in world coordinates
        """
        points_3d = []

        # Process each laser beam in the scan
        for r, alpha in zip(scan.ranges, scan.angles):
            # Skip invalid measurements
            if r <= 0 or np.isnan(r) or np.isinf(r):
                continue

            # Calculate how far the point is horizontally from the robot
            horizontal_dist = r * np.cos(alpha)

            # Transform to world coordinates using robot's pose
            x_3d = pose.x + horizontal_dist * np.cos(pose.theta)
            y_3d = pose.y + horizontal_dist * np.sin(pose.theta)
            z_3d = self.vertical_offset + r * np.sin(alpha)

            points_3d.append(Point3D(x_3d, y_3d, z_3d))

        return points_3d


# =============================================================================
# MAIN SYSTEM CLASS
# =============================================================================

class DualLiDARSLAM:
    """
    Coordinates the 3D mapping process

    NOTE: This does NOT do the 2D SLAM! That's done by slam_toolbox.
    This class only combines slam_toolbox's output with vertical LiDAR data.
    """

    def __init__(self, vertical_offset: float = 0.15):
        """
        Args:
            vertical_offset: Height of vertical LiDAR above ground (meters)
        """
        self.transformer = VerticalLiDARTransformer(vertical_offset)
        self.point_cloud_3d: List[Point3D] = []  # All 3D points we've collected

    def add_scan_with_pose(self, scan: LiDARScan, pose: Pose2D):
        """
        Process one vertical scan and add its 3D points to our map

        Args:
            scan: Vertical LiDAR scan
            pose: Robot pose from slam_toolbox at same time as scan
        """
        new_points = self.transformer.transform_scan_to_3d(scan, pose)
        self.point_cloud_3d.extend(new_points)

    def get_point_cloud(self) -> np.ndarray:
        """Get all 3D points as a numpy array (N x 3)"""
        if not self.point_cloud_3d:
            return np.array([]).reshape(0, 3)
        return np.array([[p.x, p.y, p.z] for p in self.point_cloud_3d])

    def export_point_cloud(self, filename: str, format: str = "ply"):
        """
        Save the 3D point cloud to a file

        Args:
            filename: Output file path
            format: Either "xyz" or "ply"
        """
        points = self.get_point_cloud()

        if format == "xyz":
            # Simple text format: each line is "x y z"
            np.savetxt(filename, points, fmt='%.6f')
        elif format == "ply":
            # Standard 3D model format (can open in MeshLab, CloudCompare, etc.)
            self._export_ply(filename, points)
        else:
            raise ValueError(f"Unsupported format: {format}")

    def _export_ply(self, filename: str, points: np.ndarray):
        """Write PLY format (common 3D point cloud format)"""
        with open(filename, 'w') as f:
            # PLY header
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")

            # Point data
            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")


# =============================================================================
# ROS BAG PROCESSING
# =============================================================================

# Register ROS2 LaserScan message type so we can read it from bag files
MSG_TYP = """
std_msgs/Header header
    builtin_interfaces/Time stamp
        int32 sec
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


def load_poses(pose_file: str) -> pd.DataFrame:
    """
    Load robot poses from the CSV file created by pose_listener.py

    This file contains the output from slam_toolbox:
    - timestamp: when the pose was recorded
    - x, y: robot position in meters
    - theta: robot orientation in radians

    Returns:
        DataFrame with timestamp as index
    """
    print(f"Loading poses from {pose_file}...")
    try:
        df = pd.read_csv(pose_file)
    except FileNotFoundError:
        print(f"ERROR: Pose file not found at {pose_file}")
        print("Please run the pose_listener.py script first to collect poses.")
        exit(1)

    df = df.sort_values(by='timestamp')
    df = df.set_index('timestamp')
    print(f"Loaded {len(df)} poses")
    return df


def extract_vertical_scans(bag_file: str, vertical_topic: str) -> List[LiDARScan]:
    """
    Extract all vertical LiDAR scans from a ROS2 bag file

    Args:
        bag_file: Path to .bag file (recorded with ros2 bag record)
        vertical_topic: ROS topic name for vertical LiDAR (e.g., "/scan_vertical")

    Returns:
        List of all scans found in the bag
    """
    print(f"Extracting vertical scans from {bag_file} on topic {vertical_topic}...")
    scans = []

    with AnyReader([Path(bag_file)]) as reader:
        # Check if the topic exists in the bag
        if vertical_topic not in reader.topics:
            print(f"ERROR: Vertical topic {vertical_topic} not found in {bag_file}")
            print(f"Available topics: {list(reader.topics.keys())}")
            exit(1)

        # Read all messages from the vertical LiDAR topic
        connections = [x for x in reader.connections if x.topic == vertical_topic]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # Deserialize the ROS message
            msg = reader.deserialize(rawdata, connection.msgtype)

            # Extract timestamp from message header
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Extract range measurements
            ranges = np.array(msg.ranges, dtype=np.float32)

            # Calculate angle for each measurement
            # LiDAR scans from angle_min to angle_max in steps of angle_increment
            if hasattr(msg, 'angle_min') and hasattr(msg, 'angle_increment'):
                num = len(ranges)
                angles = msg.angle_min + np.arange(num, dtype=np.float32) * msg.angle_increment
                angles = angles.astype(np.float32)
            else:
                angles = np.array([], dtype=np.float32)

            # Create a scan object and add to list
            scans.append(LiDARScan(ranges, angles, timestamp))

    print(f"Extracted {len(scans)} vertical scans")
    return scans


def find_closest_pose(timestamp: float, pose_df: pd.DataFrame, max_dt: float = 0.1) -> Optional[Pose2D]:
    """
    Find the robot pose closest in time to a LiDAR scan

    Args:
        timestamp: Time of the LiDAR scan
        pose_df: DataFrame of poses from slam_toolbox
        max_dt: Maximum time difference allowed (seconds)
               If no pose within this window, return None

    Returns:
        The closest pose, or None if no pose is close enough
    """
    if pose_df is None or len(pose_df) == 0:
        return None

    # Binary search to find insertion point
    idx = pose_df.index.searchsorted(timestamp)

    # Check poses before and after this point
    candidates = []
    if idx < len(pose_df):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)

    # Find the closest one
    best_idx = None
    best_dt = None
    for i in candidates:
        dt = abs(pose_df.index[i] - timestamp)
        if best_dt is None or dt < best_dt:
            best_dt = dt
            best_idx = i

    # Reject if too far in time
    if best_idx is None or best_dt is None or best_dt > max_dt:
        return None

    # Create Pose2D object
    pose_series = pose_df.iloc[best_idx]
    return Pose2D(
        x=float(pose_series['x']),
        y=float(pose_series['y']),
        theta=float(pose_series['theta']),
        timestamp=float(pose_series.name)
    )


# =============================================================================
# MAIN PROCESSING PIPELINE
# =============================================================================

def main():
    """
    Main function - processes recorded data to create 3D map

    REQUIRED FILES (from data collection):
    1. poses_log.txt - from pose_listener.py (slam_toolbox output)
    2. bag file - recorded vertical LiDAR data

    OUTPUT:
    - 3D point cloud file (.ply or .xyz)
    """

    # =========================================================================
    # CONFIGURATION - SET THESE VALUES FOR YOUR SETUP
    # =========================================================================

    BAG_FILE = ""
    POSE_FILE = "poses_log.txt"
    VERTICAL_SCAN_TOPIC = "/scan_vertical"  # Or whatever your vertical LiDAR publishes to
    VERTICAL_OFFSET_METERS = 0.15  # Height of vertical LiDAR above ground in meters (Default, change if not the same)
    OUTPUT_FILE = "my3dmap.ply"  # Output filename

    # =========================================================================
    # STEP 1: Load poses from slam_toolbox
    # =========================================================================
    print("\n" + "=" * 70)
    print("STEP 1: Loading 2D poses from slam_toolbox")
    print("=" * 70)
    pose_df = load_poses(POSE_FILE)

    # =========================================================================
    # STEP 2: Extract vertical LiDAR scans from bag
    # =========================================================================
    print("\n" + "=" * 70)
    print("STEP 2: Extracting vertical LiDAR scans")
    print("=" * 70)
    scan_data = extract_vertical_scans(BAG_FILE, VERTICAL_SCAN_TOPIC)

    if not scan_data:
        print("ERROR: No scan data found. Exiting.")
        return

    # =========================================================================
    # STEP 3: Combine scans and poses to build 3D map
    # =========================================================================
    print("\n" + "=" * 70)
    print("STEP 3: Building 3D point cloud")
    print("=" * 70)
    print("Matching scans with poses and transforming to 3D...")

    slam = DualLiDARSLAM(vertical_offset=VERTICAL_OFFSET_METERS)
    scans_processed = 0
    scans_skipped = 0

    for scan in tqdm(scan_data, desc="Processing scans"):
        # Find the pose closest in time to this scan
        pose = find_closest_pose(scan.timestamp, pose_df)

        if pose:
            # We have a matching pose - transform scan to 3D
            slam.add_scan_with_pose(scan, pose)
            scans_processed += 1
        else:
            # No pose found within time window - skip this scan
            scans_skipped += 1

    # =========================================================================
    # STEP 4: Export results
    # =========================================================================
    print("\n" + "=" * 70)
    print("STEP 4: Exporting 3D map")
    print("=" * 70)
    print(f"Total scans processed: {scans_processed}")
    print(f"Total scans skipped (no matching pose): {scans_skipped}")

    total_points = len(slam.point_cloud_3d)

    if total_points > 0:
        print(f"\nExporting {total_points} points to {OUTPUT_FILE}...")
        slam.export_point_cloud(OUTPUT_FILE, format="ply")
        print(f"Success! You can now open {OUTPUT_FILE}")
    else:
        print("\n✗ ERROR: No points were generated. The map is empty.")


if __name__ == "__main__":
    main()