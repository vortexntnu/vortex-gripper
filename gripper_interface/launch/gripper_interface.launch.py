from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

params = path.join(
    get_package_share_directory('gripper_interface'),
    'config',
    'params.yaml'
)
def generate_launch_description():
    gripper_interface_node = Node(
            package='gripper_interface',
            executable='gripper_interface_node',
            name='gripper_interface_node',
            parameters=[params],
            output='screen',
        )
    return LaunchDescription([
        gripper_interface_node
    ])