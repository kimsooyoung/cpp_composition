import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction, RegisterEventHandler


def generate_launch_description():

    server_node = Node(
        package='cpp_camera_composition',
        node_executable='server',
        node_namespace='ns',
        node_name='server_node',
        output='screen',
        parameters=[{
            "service_name": "add_two_ints",
        }],
    )

    client_node = Node(
        package='cpp_camera_composition',
        node_executable='client',
        node_namespace='ns',
        node_name='client_node',
        output='screen',
        arguments=["1", "2"],
        parameters=[{
            "service_name": "add_two_ints",
        }],
    )

    return launch.LaunchDescription([
        server_node, 
        TimerAction(    
            period=3.0,
            actions=[client_node]
        ),
    ])
