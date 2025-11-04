#!/usr/bin/env python3
"""
Launch file for Virtual Pet nodes only (without Gazebo)
Use this if you want to start Gazebo separately
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Gesture Recognizer Node
        Node(
            package='virtual_pet',
            executable='gesture_recognizer',
            name='gesture_recognizer',
            output='screen',
            emulate_tty=True,
        ),
        
        # Shape Drawer Node
        Node(
            package='virtual_pet',
            executable='shape_drawer',
            name='shape_drawer',
            output='screen',
            emulate_tty=True,
        ),
        
        # Pet Controller Node
        Node(
            package='virtual_pet',
            executable='pet_controller',
            name='pet_controller',
            output='screen',
            emulate_tty=True,
        ),
    ])
