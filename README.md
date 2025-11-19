Step 1: Install Dependencies
You need Python packages and ROS 2 packages:
 ROS 2 packages
sudo apt install ros-<ros2-distro>-rosbag2 ros-<ros2-distro>-sensor-msgs ros-<ros2-distro>-pcl-ros

Python packages
pip install numpy sensor-msgs sensor-msgs-py
Replace <ros2-distro> with your ROS 2 version (e.g., humble, galactic).
Step 2: Set Up Cartographer ROS
If you haven’t already, clone and build cartographer_ros:
cd ~/ros2_ws/src
git clone https://github.com/cartographer-project/cartographer_ros.git
cd ..
colcon build --symlink-install
source install/setup.bash
Make sure your workspace is sourced:
source ~/ros2_ws/install/setup.bash
Step 3: Configure Cartographer for 3D
You need a Lua config file, e.g., my_robot_3d.lua, in your cartographer_ros config folder. Key parameters:
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 30.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
And make sure the topic matches your publisher:
TRAJECTORY_BUILDER_3D.laser_topic = "/points_raw"
Set map_frame, odom_frame, and tracking_frame according to your robot:
map_frame = "map"
tracking_frame = "base_link"
odom_frame = "odom"
provide_odom_frame = true
Step 4: Prepare Python Publisher
You already have your script that:
Reads the bag.
Converts horizontal & vertical laser scans into 3D points.
Publishes them as PointCloud2 on /points_raw.
✅ Make sure you include:
from std_msgs.msg import Header
And in your numpy_to_pointcloud2 function:
header = Header()
header.stamp = rclpy.time.Time().to_msg()
header.frame_id = frame_id
Step 5: Run the Python Publisher
Run your Python script with ROS 2:
ros2 run <your_package_name> bag_pointcloud_publisher.py
This will publish the point clouds at 10 Hz (or whatever interval you set).
Make sure /points_raw matches the topic in your Cartographer Lua file.
Step 6: Launch Cartographer 3D
Use the standard launch file:
ros2 launch cartographer_ros cartographer.launch.py \
  configuration_directory:=<path_to_config> \
  configuration_basename:=my_robot_3d.lua
This will subscribe to /points_raw and build a 3D map.
Cartographer will also listen to /odom if you provide odometry in the bag.
Step 7: Visualize in RViz
Open RViz:
ros2 run rviz2 rviz2
Add Map display and set the topic to /map (Cartographer publishes it).
Optionally, add PointCloud2 display to see your published points.
Step 8: Optional – Save the Map
Once the SLAM has run:
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=your_map.pbstream \
  -map_filename=your_map
This saves the 2D/3D map for later use.

