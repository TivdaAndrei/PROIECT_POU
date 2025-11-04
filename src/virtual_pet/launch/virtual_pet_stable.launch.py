#!/usr/bin/env python3
"""
Launch file for Virtual Pet - Stable Version
Launches Gazebo headless first, then GUI separately to avoid crashes
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get TurtleBot3 model from environment or default to burger
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Start Gazebo server (headless - more stable)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '-v2',
                 PathJoinSubstitution([
                     FindPackageShare('turtlebot3_gazebo'),
                     'worlds',
                     'empty_world.sdf'
                 ]),
                 '--force-version', '8'],
            output='screen',
            shell=False
        ),
        
        # Wait for Gazebo to start
        TimerAction(
            period=3.0,
            actions=[
                # Spawn TurtleBot3
                ExecuteProcess(
                    cmd=['gz', 'service', '-s', '/world/empty/create',
                         '--reqtype', 'gz.msgs.EntityFactory',
                         '--reptype', 'gz.msgs.Boolean',
                         '--timeout', '1000',
                         '--req', f'sdf_filename: "{PathJoinSubstitution([FindPackageShare("turtlebot3_gazebo"), "models", "turtlebot3_burger", "model.sdf"])}"'],
                    output='screen',
                    shell=False
                ),
            ]
        ),
        
        # Launch pet nodes after Gazebo stabilizes
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
            ]
        ),
    ])
