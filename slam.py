import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

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
    remove = False

    while reader.has_next():
        topic, data, recv_ts = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg_deserialized = deserialize_message(data, msg_type)
        
        if isinstance(msg_deserialized, Odometry) and topic == "odom":
            if not remove:
                odom_info.append((recv_ts, msg_deserialized))
                print(recv_ts, "odom")
            remove = not remove
            

        if isinstance(msg_deserialized, LaserScan):
            if topic == "horizontal":
                hor_lidar_info.append((recv_ts, msg_deserialized))
                print(recv_ts, "horizontal")
            elif topic == "vertical":
                ver_lidar_info.append((recv_ts, msg_deserialized))
                print(recv_ts, "vertical")

    for i, (time, odom) in enumerate(odom_info):
        frame_cloud = []
        pose = odom.pose.pose

        cur_x = pose.position.x
        cur_y = pose.position.y
        cur_z = pose.position.z
        robot_coor = [cur_x, cur_y, cur_z]

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        cur_t = yaw
        
        hor_data = hor_lidar_info[i]
        hor_angle_min = hor_data[1].angle_min
        hor_angle_max = hor_data[1].angle_max
        hor_angle_increment = hor_data[1].angle_increment
        hor_ranges = hor_data[1].ranges     


        hor_rotation = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        hor_translation = np.array([
            [robot_coor[0]],
            [robot_coor[1]],
            [robot_coor[2] + robot_height/2 + laser_height/2],
            [1]
        ])

        rotation_zeros = np.zeros((hor_rotation.shape[1]))
        translation_one = np.ones((hor_translation.shape[1]))

        # trans_hor2odom = np.concatenate(np.append(hor_rotation, rotation_zeros, axis=0), np.append(hor_translation, translation_one, axis=0), axis=-1)
        trans_hor2odom = np.eye(4)
        trans_hor2odom[0:3, 0:3] = hor_rotation
        trans_hor2odom[0:3, 3] = hor_translation[0:3,0]

        vertical_rotation = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ])
        ver_rotation = hor_rotation @ vertical_rotation
        ver_translation = np.array([
            [hor_translation[0,0]],
            [hor_translation[1,0]],
            [hor_translation[2,0] + laser_height/2 + laser_radius]
        ])

        # trans_ver2odom = np.concatenate(np.append(ver_rotation, rotation_zeros, axis=0), np.append(ver_translation, translation_one, axis=0), axis=-1)
        trans_ver2odom = np.eye(4)
        trans_ver2odom[0:3, 0:3] = ver_rotation
        trans_ver2odom[0:3, 3] = ver_translation[0:3,0]

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

            point_transformed = trans_hor2odom @ point
            frame_cloud.append(point_transformed[:3])
          

            hor_ranges_xy.append(point_transformed)
            angle = angle + hor_angle_increment


        ver_data = ver_lidar_info[i]
        ver_angle_min = ver_data[1].angle_min
        ver_angle_max = ver_data[1].angle_max
        ver_angle_increment = ver_data[1].angle_increment
        ver_ranges = ver_data[1].ranges

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

            point_transformed = trans_ver2odom @ point
            frame_cloud.append(point_transformed[:3])
            ver_ranges_xz.append(point_transformed)
            angle = angle + ver_angle_increment
        all_pointclouds.append(np.array(frame_cloud))
    return all_pointclouds
#Dependencies
#Make sure you have these installed:
#ros2 pkg install sensor_msgs
#ros2 pkg install pcl_ros  # optional, if using PCL
#Python packages:
#pip install numpy

#Topic Name
#In your Cartographer .launch or Lua config, set:
#TRAJECTORY_BUILDER_3D.laser_min_range = 0.1
#TRAJECTORY_BUILDER_3D.laser_max_range = 30.0
#TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

#-- Your ROS topic:
#TRAJECTORY_BUILDER_3D.laser_topic = "/points_raw"
#Frame
#Make sure frame_id matches your Cartographer config (map, odom, or base_link).
#Publishing Rate
#The 0.1 timer interval in Python simulates a 10 Hz point cloud stream. Adjust to match your original sensor rate if needed.

def numpy_to_pointcloud2(points, frame_id="map"):
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
    def __init__(self, frame_clouds, topic_name="/points_raw", frame_id="map"):
        super().__init__("bag_pointcloud_publisher")
        self.pub = self.create_publisher(PointCloud2, topic_name, 10)
        self.frame_clouds = frame_clouds
        self.frame_id = frame_id
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.index = 0

    def timer_callback(self):
        if self.index >= len(self.frame_clouds):
            self.get_logger().info("Finished publishing all frames")
            return

        points = np.array(self.frame_clouds[self.index])
        pc2_msg = numpy_to_pointcloud2(points, self.frame_id)
        self.pub.publish(pc2_msg)
        self.get_logger().info(f"Published frame {self.index} with {points.shape[0]} points")
        self.index += 1
def main(args=None):
    import rclpy
    from read_bag_your_code import read_bag  # your existing function

    rclpy.init(args=args)

    # Read rosbag and get frame clouds
    frame_clouds = read_bag("/home/agathe/Documents/york/fall2025/robotics/project_ros_ws/test_bag")

    node = BagPointCloudPublisher(frame_clouds)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

