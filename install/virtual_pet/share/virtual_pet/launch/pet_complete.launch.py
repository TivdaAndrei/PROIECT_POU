#!/usr/bin/env python3
"""
Complete launch with GUI and RViz for trail visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get our custom RViz config path
    rviz_config_dir = os.path.join(
        get_package_share_directory('virtual_pet'),
        'rviz',
        'trail_view.rviz'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # GUI Controller Node
        Node(
            package='virtual_pet',
            executable='gui_controller',
            name='gui_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}]
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
        
        # Voice Chat Node
        Node(
            package='virtual_pet',
            executable='voice_chat',
            name='voice_chat',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # RViz for trail visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
