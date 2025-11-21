import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/agathe/Documents/york/fall2025/robotics/project_ros_ws/ydlidar_ros2_ws/install/simulation'
