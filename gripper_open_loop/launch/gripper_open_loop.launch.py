import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("gripper_open_loop"),
        "config",
        "gripper_open_loop_params.yaml",
    )

    container = ComposableNodeContainer(
        name="gripper_open_loop_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="gripper_open_loop",
                plugin="vortex::open_loop::GripperOpenLoopNode",
                name="gripper_open_loop_node",
                parameters=[config],
            )
        ],
        output="screen",
    )

    return LaunchDescription([container])
