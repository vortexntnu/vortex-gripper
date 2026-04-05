import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('gripper_interface'),
        'config',
        'gripper_interface_params.yaml',
    )

    gripper_interface_node = Node(
        package='gripper_interface',
        executable='gripper_interface_standalone',
        name='gripper_interface_node',
        parameters=[config_file_path],
        output='screen',
    )

    return LaunchDescription([gripper_interface_node])
