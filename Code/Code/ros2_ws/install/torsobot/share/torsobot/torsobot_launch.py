from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path

# Find the YAML file of parameters
params_file_name = 'params.yaml'
params_file = os.path.join(
    '/home/pi/torsobot/Code/Code/ros2_ws/src/torsobot/config/',
    params_file_name
)

# /home/pi/torsobot/Code/Code/ros2_ws/install/torsobot/share/torsobot/params.yaml


def generate_launch_description():
    return LaunchDescription([
        Node(package='torsobot',
             executable='mcu_node', parameters=[params_file]),
        Node(package='torsobot',
             executable='data_logger_node', parameters=[params_file])
    ])
