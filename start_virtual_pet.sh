#!/bin/bash
# Virtual Pet Launcher (manual spawn)

echo "========================================"
echo "🤖 Virtual Pet Launcher"
echo "========================================"

gnome-terminal --title="Gazebo" --geometry=100x30+0+0 -- bash -c '
    source /opt/ros/jazzy/setup.bash
    source ~/ROS2_ws/install/setup.bash
    export TURTLEBOT3_MODEL=burger
    echo "🌍 Starting Gazebo..."
    ros2 launch turtlebot3_gazebo empty_world.launch.py
    exec bash
' &

sleep 3

gnome-terminal --title="Virtual Pet" --geometry=100x30+800+0 -- bash -c '
    source /opt/ros/jazzy/setup.bash
    source ~/ROS2_ws/install/setup.bash
    export TURTLEBOT3_MODEL=burger
    echo "⏳ Waiting for Gazebo..."
    sleep 8
    echo "🚀 Starting Virtual Pet..."
    ros2 launch virtual_pet pet_complete.launch.py
    exec bash
' &

echo ""
echo "✅ Terminals opened!"
echo ""
echo "⏳ Wait 10 seconds, then spawn robot:"
echo "ros2 run gazebo_ros spawn_entity.py -entity turtlebot3 -file ~/ROS2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf -x 0 -y 0 -z 0.01"
echo ""
echo "🎤 Say 'Hi friend' to start!"
echo "========================================"
