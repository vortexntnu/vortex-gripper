from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_sim_interface',
            executable='gripper_sim_bridge',
            name='gripper_sim_bridge',
            output='screen',
        )
    ])
