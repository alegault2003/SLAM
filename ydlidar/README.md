# YDLiDAR

The ydlidar folder is used to collect data from the LiDAR sensors, YDLiDAR G4. It has `ydlidar_ros_ws`, which is used used as the driver of the senors and uses the other folder `YDLidar-SDK` as a library.

The base code along with their docs can be found at:
- `YDLidar-SDK`: https://github.com/YDLIDAR/YDLidar-SDK.git
- `ydlidar_ros_ws`: https://github.com/YDLIDAR/ydlidar_ros2_driver.git

## Set up

### build and install YDLidar-SDK

first step is to install all `cmake`, which is used to build and install the library.
```bash
sudo apt install cmake pkg-config
```

Next is to build and install the library. Use the following commands to do so:
```bash
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

### set up and build ydlidar_ros_ws

First, make sure that the ros workspace is able to build. Use the command below.
```bash
colcon build --symlink-install
```
If the command results in some code warnings (cpp code warnings should be purple in terminal), this is ok. Run the command again and the warnings should no longer come up.

Next, the code must be configured to collect the right data (horizontal or vertical) from the correct LiDAR sensor. 

The device port `/dev/ttyUSB*` must be found for each LiDAR. To do so, keep both sensors unplugged from the computer. Run the command below to clear the kernel buffer.
```bash
sudo dmesg -C
```
Plug in one of the sensors through USB and run
```bash
sudo dmesg
```
The output should show that a new USB device has been connected and will show what `/dev/ttyUSB*` it was connected to. Take note of it and use the same commands for the other sensor.

With the device ports found, we will configure some of the driver files. Head to the configuration file `/ydlidar_ros_ws/src/ydlidar_ros_driver/src/horizontal_driver_node.cpp`. This is the ROS node that will collect information from the horizontal LiDAR sensor and publish it to a ROS topic. On line 43 of the file, you'll see the code 
```
std::string str_optvalue = "/dev/ttyUSB1";
```
Set the value of `str-optvalue` to the device port for the horizontal LiDAR. 

Furthermore, head to `/ydlidar_ros_ws/src/ydlidar/params/ydlidar_horizontal.yaml`. You will once again change the device port on line 3
```
port: /dev/ttyUSB1
```

repeat the previous two steps for the vertical LiDAR.

## Run

With everything set up, build the `ydlidar_ros_ws` with the command `colcon build --symlink-install` and then source it with `source install/setup.bash`

To run the LiDARs and collect the data, we will need 4 terminals.
NOTE: all commands should be done in the `ydlidar_ros_ws` directory

The first and second windows will be used to launch the horizontal and vertical driver nodes, using the commands
```bash
ros2 launch ydlidar_ros_driver horizontal_lidar.launch.py

ros2 launch ydlidar_ros_driver vertical_lidar.launch.py
```

The third terminal will be used to start the scans of each LiDAR.
```bash
ros2 service call /horizontal/start_scan std_srvs/srv/Empty "{}"

ros2 service call /vertical/start_scan std_srvs/srv/Empty "{}"
```

Finally, the fourth terminal will be used to run the data collection node.
```bash
ros2 launch ydlidar_ros_driver collect_data.launch.py
```

The output of the `collect_data.launch.py` node once it is no longer running is a rosbag named `test_both` that uses `mcap` storage.

To change the name and location of the resulting bag, head to the file `ydlidar_ros_ws/src/ydlidar_ros_driver/ydlidar_ros_driver/collect_data.py`. In line 50, in the `StorageOptions` function, change the value of `uri` to the desired location/name. (make sure to build and source the workspace again before running the nodes)
