#!/bin/bash#!/bin/bash

# Virtual Pet Automatic Launcher# Quick start script for Virtual Pet project

# Opens two terminals and launches the complete system

# Colors for output

echo "=========================================="GREEN='\033[0;32m'

echo "🤖 Starting Virtual Pet System"BLUE='\033[0;34m'

echo "=========================================="YELLOW='\033[1;33m'

echo ""NC='\033[0m' # No Color



# Check if ROS2 is sourcedecho -e "${BLUE}╔════════════════════════════════════════╗${NC}"

if [ -z "$ROS_DISTRO" ]; thenecho -e "${BLUE}║     Virtual Pet - Quick Start 🐾      ║${NC}"

    echo "⚠️  ROS2 environment not sourced!"echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"

    echo "Sourcing ROS2 Jazzy..."echo ""

    source /opt/ros/jazzy/setup.bash

fi# Source ROS2 workspace

echo -e "${YELLOW}Sourcing ROS2 workspace...${NC}"

# Source workspacesource /home/andrei-robert/ROS2_ws/install/setup.bash

if [ -f "$HOME/ROS2_ws/install/setup.bash" ]; then

    echo "✅ Sourcing workspace..."# Set TurtleBot3 model

    source $HOME/ROS2_ws/install/setup.bashexport TURTLEBOT3_MODEL=burger

elseecho -e "${GREEN}✓ TurtleBot3 model set to: $TURTLEBOT3_MODEL${NC}"

    echo "❌ Workspace not built! Run 'colcon build' first."

    exit 1# Check if Python packages are installed

fiecho -e "${YELLOW}Checking Python dependencies...${NC}"

/home/andrei-robert/ROS2_ws/.venv/bin/python -c "import cv2, mediapipe" 2>/dev/null

# Set TurtleBot3 modelif [ $? -eq 0 ]; then

export TURTLEBOT3_MODEL=burger    echo -e "${GREEN}✓ Python dependencies OK${NC}"

echo "✅ TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"else

    echo -e "${YELLOW}⚠ Installing Python dependencies...${NC}"

echo ""    /home/andrei-robert/ROS2_ws/.venv/bin/pip install -r /home/andrei-robert/ROS2_ws/src/virtual_pet/requirements.txt

echo "Opening terminals..."fi

echo ""

echo ""

# Terminal 1: Gazeboecho -e "${BLUE}Starting Virtual Pet...${NC}"

gnome-terminal --title="Gazebo Simulator" --geometry=100x30+0+0 -- bash -c "echo -e "${YELLOW}Hand Gesture Controls:${NC}"

    echo '=========================================='echo "  ✌️  Peace (2 fingers) → Print 'PACE!' & celebrate"

    echo '🌍 Gazebo Simulator'echo "  ✊  Fist → Draw Square"

    echo '=========================================='echo "  ✋  Open Hand → Draw Circle"

    echo ''echo "  👆  One Finger → Draw Line"

    echo 'Sourcing ROS2 environment...'echo "  🤘  Rock Sign → Draw Triangle"

    source /opt/ros/jazzy/setup.bashecho "  🤙  Three Fingers → Draw Star"

    source $HOME/ROS2_ws/install/setup.bashecho ""

    export TURTLEBOT3_MODEL=burgerecho -e "${GREEN}Launching in 3 seconds...${NC}"

    echo ''sleep 3

    echo '🚀 Launching Gazebo...'

    echo ''# Launch the virtual pet

    ros2 launch turtlebot3_gazebo empty_world.launch.pyros2 launch virtual_pet virtual_pet.launch.py

    exec bash
" &

# Wait a bit for Gazebo to start
sleep 3

# Terminal 2: Virtual Pet System (ROS2 nodes + RViz + GUI + Voice)
gnome-terminal --title="Virtual Pet System" --geometry=100x30+800+0 -- bash -c "
    echo '=========================================='
    echo '🤖 Virtual Pet Control System'
    echo '=========================================='
    echo ''
    echo 'Sourcing ROS2 environment...'
    source /opt/ros/jazzy/setup.bash
    source $HOME/ROS2_ws/install/setup.bash
    export TURTLEBOT3_MODEL=burger
    echo ''
    echo '⏳ Waiting for Gazebo to be ready...'
    sleep 5
    echo ''
    echo '🚀 Launching Virtual Pet nodes...'
    echo '   - Gesture Recognition'
    echo '   - Shape Drawer'
    echo '   - Pet Controller'
    echo '   - GUI Controller'
    echo '   - Voice Chat'
    echo '   - RViz Visualization'
    echo ''
    echo '📢 Say \"Hi friend\" to start chatting!'
    echo ''
    ros2 launch virtual_pet pet_complete.launch.py
    exec bash
" &

echo ""
echo "=========================================="
echo "✅ System Starting!"
echo "=========================================="
echo ""
echo "📺 Terminal 1: Gazebo (left)"
echo "🤖 Terminal 2: Virtual Pet (right)"
echo ""
echo "⏳ Wait ~10 seconds for everything to load"
echo ""
echo "Then spawn the robot with:"
echo "  ros2 run gazebo_ros spawn_entity.py -entity turtlebot3 \\"
echo "    -file ~/ROS2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \\"
echo "    -x 0 -y 0 -z 0.01"
echo ""
echo "🎤 Voice Commands:"
echo "  - Wake word: 'Hi friend'"
echo "  - Chat naturally about anything"
echo "  - Say shape names to draw: circle, square, triangle, star, line"
echo ""
echo "👋 Have fun with your virtual pet!"
echo "=========================================="
