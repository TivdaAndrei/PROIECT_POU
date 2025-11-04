#!/usr/bin/env python3
"""
Launch file for Virtual Pet with TurtleBot3 in Gazebo
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get TurtleBot3 model from environment or default to burger
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Path to TurtleBot3 Gazebo launch file
    turtlebot3_gazebo_launch = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    ])
    
    return LaunchDescription([
        # Set TurtleBot3 model environment variable
        ExecuteProcess(
            cmd=['echo', f'Using TurtleBot3 model: {turtlebot3_model}'],
            output='screen'
        ),
        
        # Launch TurtleBot3 in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_gazebo_launch),
            launch_arguments={'x_pose': '0.0',
                            'y_pose': '0.0'}.items()
        ),
        
        # Wait a bit for Gazebo to start, then launch pet nodes
        TimerAction(
            period=5.0,
            actions=[
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
            ]
        ),
    ])
