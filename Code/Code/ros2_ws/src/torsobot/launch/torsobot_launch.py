from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='torsobot',
             executable='mcu_node'), Node(package='torsobot',
                                          executable='data_logger_node')
    ])
