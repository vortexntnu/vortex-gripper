from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

params = path.join(
    get_package_share_directory('gripper_controller'), 'config', 'gripper_controller_config.yaml'
)


def generate_launch_description():
    gripper_controller_node = Node(
        package='gripper_controller',
        executable='gripper_controller_node',
        name='gripper_controller_node',
        parameters=[params],
        output='screen',
    )
    return LaunchDescription([gripper_controller_node])
