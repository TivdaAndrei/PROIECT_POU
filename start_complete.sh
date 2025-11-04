#!/bin/bash
# Complete Virtual Pet Startup - Ensures everything is connected

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Virtual Pet - Complete Startup 🐾   ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

cd ~/ROS2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo -e "${YELLOW}Step 1: Checking if Gazebo is running...${NC}"
if ! pgrep -x "gz" > /dev/null; then
    echo -e "${RED}❌ Gazebo is not running!${NC}"
    echo ""
    echo "Please start Gazebo in another terminal FIRST:"
    echo -e "${GREEN}  cd ~/ROS2_ws${NC}"
    echo -e "${GREEN}  source install/setup.bash${NC}"
    echo -e "${GREEN}  export TURTLEBOT3_MODEL=burger${NC}"
    echo -e "${GREEN}  ros2 launch turtlebot3_gazebo empty_world.launch.py${NC}"
    echo ""
    echo "Then manually add TurtleBot3 in Gazebo GUI (Insert tab)"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓ Gazebo is running${NC}"
echo ""

echo -e "${YELLOW}Step 2: Checking for /cmd_vel topic...${NC}"
sleep 2
if ros2 topic list | grep -q "/cmd_vel"; then
    echo -e "${GREEN}✓ /cmd_vel topic exists${NC}"
else
    echo -e "${YELLOW}⚠ /cmd_vel topic not found - robot might not be spawned${NC}"
    echo "Make sure you added TurtleBot3 in Gazebo!"
fi
echo ""

echo -e "${YELLOW}Step 3: Starting Virtual Pet nodes...${NC}"
echo ""
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo -e "${YELLOW}Hand Gesture Controls:${NC}"
echo "  ✌️  Peace (2 fingers) → Print 'PACE!' & celebrate"
echo "  ✊  Fist → Draw Square"
echo "  ✋  Open Hand → Draw Circle"
echo "  👆  One Finger → Draw Line"
echo "  🤘  Rock Sign → Draw Triangle"
echo "  🤙  Three Fingers → Draw Star"
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo ""
echo -e "${GREEN}Camera window will open in 3 seconds...${NC}"
echo -e "${YELLOW}IMPORTANT: Make sure Gazebo is NOT paused (press Spacebar in Gazebo)${NC}"
echo ""
sleep 3

ros2 launch virtual_pet pet_nodes_simple.launch.py
