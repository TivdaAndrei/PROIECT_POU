#!/bin/bash
# Improved start script for Virtual Pet project
# Handles Gazebo crashes and provides step-by-step startup

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     Virtual Pet - Improved Start 🐾   ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# Source ROS2 workspace
echo -e "${YELLOW}Sourcing ROS2 workspace...${NC}"
source /home/andrei-robert/ROS2_ws/install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
echo -e "${GREEN}✓ TurtleBot3 model set to: $TURTLEBOT3_MODEL${NC}"

# Kill any existing Gazebo processes
echo -e "${YELLOW}Cleaning up any existing Gazebo processes...${NC}"
killall -9 gz gzserver gzclient ruby 2>/dev/null
sleep 2
echo -e "${GREEN}✓ Clean slate${NC}"

echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${YELLOW}Due to Gazebo GUI stability issues, we'll use a 2-step approach:${NC}"
echo ""
echo -e "${GREEN}STEP 1:${NC} Start Gazebo separately (in this terminal)"
echo -e "${GREEN}STEP 2:${NC} Start pet nodes (in another terminal)"
echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# Offer choice
echo -e "${YELLOW}Choose your startup method:${NC}"
echo "  1) Start EMPTY Gazebo world (recommended - most stable)"
echo "  2) Start TurtleBot3 World (may have GUI issues)"
echo "  3) Just start the pet nodes (if Gazebo is already running)"
echo "  4) Try full auto-launch (may crash)"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo -e "${GREEN}Starting empty Gazebo world...${NC}"
        echo -e "${YELLOW}After Gazebo opens:${NC}"
        echo "  1. In Gazebo GUI, go to 'Insert' tab"
        echo "  2. Search for 'TurtleBot3' and add it to the world"
        echo "  3. Open a NEW terminal and run: ./start_pet_nodes.sh"
        echo ""
        sleep 3
        ros2 launch turtlebot3_gazebo empty_world.launch.py
        ;;
    2)
        echo ""
        echo -e "${GREEN}Starting TurtleBot3 world...${NC}"
        echo -e "${RED}⚠ Warning: This may crash due to GUI issues${NC}"
        echo -e "${YELLOW}If it crashes, try option 1 instead${NC}"
        echo ""
        sleep 3
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        ;;
    3)
        echo ""
        echo -e "${GREEN}Starting Virtual Pet nodes only...${NC}"
        echo ""
        echo -e "${YELLOW}Hand Gesture Controls:${NC}"
        echo "  ✌️  Peace (2 fingers) → Print 'PACE!' & celebrate"
        echo "  ✊  Fist → Draw Square"
        echo "  ✋  Open Hand → Draw Circle"
        echo "  👆  One Finger → Draw Line"
        echo "  🤘  Rock Sign → Draw Triangle"
        echo "  🤙  Three Fingers → Draw Star"
        echo ""
        sleep 2
        ros2 launch virtual_pet pet_nodes_simple.launch.py
        ;;
    4)
        echo ""
        echo -e "${YELLOW}Attempting full auto-launch...${NC}"
        echo -e "${RED}⚠ This might crash. Be ready to Ctrl+C${NC}"
        sleep 3
        ros2 launch virtual_pet virtual_pet.launch.py
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac
