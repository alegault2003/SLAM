# SLAM

This is a EECS 4421/5324 Intro to Robotics project.

This project consists of created a 3D map of an environment using two 2D LiDAR sensors. One of the sensors will obtain a horizontal scan of the environment and the other will obtain a vertical scan.

We first start by creating a Simulated environment to create the necessary code to conduct the real-life SLAM. The code consists of scripts for data collection, a data preprocessing, fusing the data from each lidar together, and the SLAM algorithm.

## Project Ros Workspace
This ros workspace is used for the Simulation and Slam packages

It requires Gazebo and rosbag2

Please visit its README for further information

## YDLiDAR
This ros workspace is used to collect data from the LiDAR sensors, YDLiDAR G4.

It uses the YDLiDAR library "YDLiDAR-SDK," which will need to be built and installed.

It requires rosbag2

Please visit its README for further information

