#!/bin/bash#!/bin/bash

# Complete Virtual Pet Launcher - Opens terminals AND spawns robot automatically# Complete Virtual Pet Startup - Ensures everything is connected



echo "=========================================="# Colors

echo "🤖 Virtual Pet - Complete Auto Launcher"GREEN='\033[0;32m'

echo "=========================================="BLUE='\033[0;34m'

echo ""YELLOW='\033[1;33m'

RED='\033[0;31m'

# Source everythingNC='\033[0m'

source /opt/ros/jazzy/setup.bash 2>/dev/null

source $HOME/ROS2_ws/install/setup.bash 2>/dev/nullecho -e "${BLUE}╔════════════════════════════════════════╗${NC}"

export TURTLEBOT3_MODEL=burgerecho -e "${BLUE}║   Virtual Pet - Complete Startup 🐾   ║${NC}"

echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"

echo "✅ Environment configured"echo ""

echo ""

echo "Opening terminals..."cd ~/ROS2_ws

echo ""source install/setup.bash

export TURTLEBOT3_MODEL=burger

# Terminal 1: Gazebo

gnome-terminal --title="🌍 Gazebo Simulator" --geometry=100x30+0+0 -- bash -c "echo -e "${YELLOW}Step 1: Checking if Gazebo is running...${NC}"

    source /opt/ros/jazzy/setup.bashif ! pgrep -x "gz" > /dev/null; then

    source $HOME/ROS2_ws/install/setup.bash    echo -e "${RED}❌ Gazebo is not running!${NC}"

    export TURTLEBOT3_MODEL=burger    echo ""

    echo '=========================================='    echo "Please start Gazebo in another terminal FIRST:"

    echo '🌍 Gazebo Simulator'    echo -e "${GREEN}  cd ~/ROS2_ws${NC}"

    echo '=========================================='    echo -e "${GREEN}  source install/setup.bash${NC}"

    echo ''    echo -e "${GREEN}  export TURTLEBOT3_MODEL=burger${NC}"

    ros2 launch turtlebot3_gazebo empty_world.launch.py    echo -e "${GREEN}  ros2 launch turtlebot3_gazebo empty_world.launch.py${NC}"

    exec bash    echo ""

" &    echo "Then manually add TurtleBot3 in Gazebo GUI (Insert tab)"

    echo ""

sleep 3    exit 1

fi

# Terminal 2: Virtual Pet System

gnome-terminal --title="🤖 Virtual Pet System" --geometry=100x30+800+0 -- bash -c "echo -e "${GREEN}✓ Gazebo is running${NC}"

    source /opt/ros/jazzy/setup.bashecho ""

    source $HOME/ROS2_ws/install/setup.bash

    export TURTLEBOT3_MODEL=burgerecho -e "${YELLOW}Step 2: Checking for /cmd_vel topic...${NC}"

    echo '=========================================='sleep 2

    echo '🤖 Virtual Pet System'if ros2 topic list | grep -q "/cmd_vel"; then

    echo '=========================================='    echo -e "${GREEN}✓ /cmd_vel topic exists${NC}"

    echo ''else

    echo '⏳ Waiting for Gazebo (10 seconds)...'    echo -e "${YELLOW}⚠ /cmd_vel topic not found - robot might not be spawned${NC}"

    sleep 10    echo "Make sure you added TurtleBot3 in Gazebo!"

    echo '🚀 Launching all nodes...'fi

    echo ''echo ""

    ros2 launch virtual_pet pet_complete.launch.py

    exec bashecho -e "${YELLOW}Step 3: Starting Virtual Pet nodes...${NC}"

" &echo ""

echo -e "${BLUE}═══════════════════════════════════════${NC}"

sleep 5echo -e "${YELLOW}Hand Gesture Controls:${NC}"

echo "  ✌️  Peace (2 fingers) → Print 'PACE!' & celebrate"

# Terminal 3: Robot Spawner (closes automatically after spawning)echo "  ✊  Fist → Draw Square"

gnome-terminal --title="🤖 Robot Spawner" --geometry=80x20+400+400 -- bash -c "echo "  ✋  Open Hand → Draw Circle"

    source /opt/ros/jazzy/setup.bashecho "  👆  One Finger → Draw Line"

    source $HOME/ROS2_ws/install/setup.bashecho "  🤘  Rock Sign → Draw Triangle"

    export TURTLEBOT3_MODEL=burgerecho "  🤙  Three Fingers → Draw Star"

    echo '=========================================='echo -e "${BLUE}═══════════════════════════════════════${NC}"

    echo '🤖 Spawning TurtleBot3...'echo ""

    echo '=========================================='echo -e "${GREEN}Camera window will open in 3 seconds...${NC}"

    echo ''echo -e "${YELLOW}IMPORTANT: Make sure Gazebo is NOT paused (press Spacebar in Gazebo)${NC}"

    echo '⏳ Waiting for Gazebo to be ready (15 seconds)...'echo ""

    sleep 15sleep 3

    echo ''

    echo '🤖 Spawning robot...'ros2 launch virtual_pet pet_nodes_simple.launch.py

    ros2 run gazebo_ros spawn_entity.py -entity turtlebot3 \
        -file $HOME/ROS2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
        -x 0 -y 0 -z 0.01
    
    if [ \$? -eq 0 ]; then
        echo ''
        echo '✅ Robot spawned successfully!'
        echo ''
        echo '=========================================='
        echo '🎉 System Ready!'
        echo '=========================================='
        echo ''
        echo '🎤 Say \"Hi friend\" to start chatting!'
        echo '👋 Show hand gestures to the camera'
        echo '🖱️ Use GUI buttons to control'
        echo '🎨 Watch trails in RViz'
        echo ''
        echo 'This window will close in 5 seconds...'
        sleep 5
    else
        echo ''
        echo '❌ Failed to spawn robot'
        echo 'Press Enter to close...'
        read
    fi
" &

echo ""
echo "=========================================="
echo "✅ Launching Complete System!"
echo "=========================================="
echo ""
echo "What's happening:"
echo "  1️⃣  Terminal 1: Gazebo Simulator (left)"
echo "  2️⃣  Terminal 2: Virtual Pet Nodes (right)"
echo "  3️⃣  Terminal 3: Auto-spawns robot (center, auto-closes)"
echo ""
echo "⏳ Total startup time: ~20 seconds"
echo ""
echo "Then you can:"
echo "  🎤 Say 'Hi friend' to wake up Pou"
echo "  💬 Chat naturally about anything"
echo "  🎨 Say shape names to draw"
echo "  👋 Show hand gestures"
echo "  🖱️ Use GUI buttons"
echo ""
echo "To stop everything: Ctrl+C in each terminal"
echo "=========================================="
