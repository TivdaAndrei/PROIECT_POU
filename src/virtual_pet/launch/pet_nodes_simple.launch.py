#!/usr/bin/env python3
"""
Simple launch - Just the pet nodes (use when Gazebo is already running)
This is the most reliable way to run the system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Gesture Recognizer Node
        Node(
            package='virtual_pet',
            executable='gesture_recognizer',
            name='gesture_recognizer',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Shape Drawer Node
        Node(
            package='virtual_pet',
            executable='shape_drawer',
            name='shape_drawer',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Pet Controller Node
        Node(
            package='virtual_pet',
            executable='pet_controller',
            name='pet_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
