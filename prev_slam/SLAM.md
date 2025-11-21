# How to use SLAM
### 1) Setup (one-time)
```bash
chmod +x pose_listener.py

source /opt/ros/humble/setup.bash
```
---
### 2) Run SLAM
First start YDLiDAR G4
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```
Second start slam_toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py
```
Third start the pose listener
```
python3 pose_listener.py
```
---
### 3) Drive the robot around
The script will continuously log poses to `pose_log.txt`. We should see updates every 50 poses. 
To stop logging when done press `Ctrl+C` or `⌘+C`.
---
### 4) Record the vertical LiDAR data
In a fourth terminal while the above are running
```bash
ros2 bag record /scan_vertical -o my_mapping_session
```
---
## Important notes
**Topic Names**: The script subcribes to `/pose` by default (`slam_toolbx` standard pose topic).
If your `slam_toolbox` published to a different topic, you can check available topics with:
```bash
ros2 topic list
ros2 topic info /pose
```
**Output Format**: Creates a CSV file with columns:
- `timestamp` - ROS timestamp in seconds
- `x`- X position in meters
- `y` - Y position in meters
- `theta` - yaw angle in radians
**Backup protection**:If `poses_log.txt` already exists, it automatically creates a backup before
overwriting.
