import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_tools',
                    plugin='image_tools::Cam2Image',
                    name='cam2image',
                    parameters=[{
                        "show_camera": True,
                        "frequency": 30,
                        "width": 640,
                        "height": 480,
                    }]),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
