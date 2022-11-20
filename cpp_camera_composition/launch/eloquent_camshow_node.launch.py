import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    cam2image_node = Node(
        package='cpp_camera_composition',
        node_executable='cam2image_node',
        node_name='cam2image_node',
        output='screen',
        parameters=[{
            "show_camera": False,
            "frequency": 30,
            "width": 640,
            "height": 480,
        }],
    )

    showimage_node = Node(
        package='cpp_camera_composition',
        node_executable='showimage_node',
        node_name='showimage_node',
        output='screen',
        parameters=[{
            "show_image": True,
            "depth": 10,
        }],
    )

    return launch.LaunchDescription([
        cam2image_node,
        showimage_node,
    ])
