import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory("gripper_controller"),
        "config",
        "gripper_controller_params.yaml",
    )

    gripper_controller_node = Node(
        package="gripper_controller",
        executable="gripper_controller_standalone",
        name="gripper_controller_node",
        parameters=[config_file_path],
        output="screen",
    )

    return LaunchDescription([gripper_controller_node])
