# Project ROS Workspace

This ROS workspace has 3 packages:
- `simulation`: this package loads an existing Gazebo world along with a robot with 2 LiDAR sensors (one horizontal, the other vertical) and also collects data from the sensor and saves it in a RosBag.
- `slam`: this package contains the necessary scripts to process the data and feed it into the slam algorithm
- `cartographer_ros`: this package is an open-sourced ROS wrapper for the cartographer slam algorithm

To build this workspace, stay in the `project_ros_ws` directory and u-se the command `colcon build` along with `source install/setup.bash` right after

## Simulation Package

To use this package and its files, the ros workspace must be built and sourced. See the above section.

After the workspace has been built and source, one of the folders from the package, `worlds`, must be copied in the install directory. While in the `project_ros_ws` directory, use the command 
```bash
cp src/simulation/worlds install/simulation/share/simulation/worlds
```

### Launch world

To launch Gazebo with a specific world file, head to the launch file located at `src/simulation/launch/2lidar-robot.launch.py`. 

In the launch file, there is a `world_path` variables that obtains the path to the desired world file. Change the name `project_world2` to the filename of the desired world and save the launch file.

Since the launch file was changed, the simulation package needs to be built and sourced again. The Gazebo world can then be built using the command below while in the `project_ros_ws` directory:
```bash
ros2 launch simulation 2lidar-robot.launch.py
```

If you would like to launch a world that is not in the worlds folder of the simulation package, please move the world file to the worlds folder along with the worlds folder in the install folder at `project_ros_ws/install/simulation/share/simulation/worlds/<world_filename>`. With the file in the correct location, follow the previous instructions to launch the new world.

## Collect data

To collect the LiDAR sensor data, 3 ROS nodes are needed:
- `2lidar-robot.launch.py` to launch the world
- `teleop_twist_keyboard` to controll the robot around the Gazebo world
- `collect_data.launch.py`

NOTE: all commands should be done in the `project_ros_ws` directory
 
First, launch the `2lidar-robot.launch.py` node with the command
```bash
ros2 launch simulation 2lidar-robot.launch.py
```

Second, launch the teleop controller to move the robot around the Gazebo world using
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Finally, launch the data collecter node using
```bash

ros2 launch simulation collect_data.launch.py`
```

The output of the `collect_data.launch.py` node once it is no longer running is a rosbag named `sim_data` that uses `mcap` storage.

To change the name and location of the resulting bag, head to the file `project_ros_ws/src/simulation/simulation/collect_data.py`. In line 50, in the `StorageOptions` function, change the value of `uri` to the desired location/name. (make sure to build and source the workspace again before running the nodes)
