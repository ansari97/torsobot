import signal
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events.process import SignalProcess

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
    get_package_prefix,
)

# workspace directory
ws_root = os.path.abspath(os.path.join(get_package_prefix("torsobot"), "..", ".."))
datalogs_path = os.path.join(ws_root, "data_logs")

# Find the YAML file of parameters
params_file_name = "params.yaml"
params_file = os.path.join(
    get_package_share_directory("torsobot"), "config", params_file_name
)

# /home/pi/torsobot/Code/Code/ros2_ws/install/torsobot/share/torsobot/params.yaml


def generate_launch_description():

    mcu_node = Node(package="torsobot", executable="mcu_node", parameters=[params_file])

    data_logger_node = Node(
        package="torsobot",
        executable="data_logger_node",
        parameters=[
            params_file,
            {"datalogs_path": datalogs_path, "param_source_file": params_file},
        ],
    )

    selective_kill = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mcu_node,
            on_exit=[
                # Emit a specific signal instead of a global shutdown
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,  # Acts exactly like Ctrl+C
                        process_matcher=lambda process: process
                        is data_logger_node,  # Target ONLY data_logger_node
                    )
                )
            ],
        )
    )
    return LaunchDescription([mcu_node, data_logger_node, selective_kill])
