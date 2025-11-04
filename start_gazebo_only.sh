#!/bin/bash
# Start Gazebo with TurtleBot3 - SIMPLE VERSION

cd ~/ROS2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "🚀 Starting Gazebo with TurtleBot3..."
echo ""
echo "After Gazebo opens, the robot should be there."
echo "If not, use Insert tab in Gazebo to add TurtleBot3 Burger"
echo ""

ros2 launch turtlebot3_gazebo empty_world.launch.py
