import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Path to the specific gripper reference filter parameters
    config_file_path = os.path.join(
        get_package_share_directory("gripper_reference_filter"),
        "config",
        "gripper_reference_filter_params.yaml", 
    )

    container = ComposableNodeContainer(
        name='gripper_reference_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gripper_reference_filter',
                # THIS PLUGIN NAME MUST MATCH YOUR NAMESPACE AND CLASS NAME EXACTLY
                plugin='vortex::guidance::GripperReferenceFilterNode', 
                name='gripper_reference_filter_node',
                parameters=[config_file_path],
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
