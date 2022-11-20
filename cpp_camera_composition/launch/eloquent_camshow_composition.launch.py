import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='my_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_tools',
                    node_plugin='image_tools::Cam2Image',
                    node_name='cam2image',
                    parameters=[{
                        "show_camera": False,
                        "frequency": 30.0,
                        "width": 640,
                        "height": 480,
                    }]
                ),
                # ComposableNode(
                #     package='image_tools',
                #     node_plugin='image_tools::ShowImage',
                #     node_name='showimage',
                #     parameters=[{
                #         "history": "keep_last",
                #     }]
                # ),
                ComposableNode(
                    package='cpp_camera_composition',
                    node_plugin='cpp_camera_composition::GrayImage',
                    node_name='grayimage',
                    parameters=[{
                        "show_camera": True,
                    }]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
