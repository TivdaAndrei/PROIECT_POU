#!/bin/bash
# Start Pet Nodes - Simple Version (use in terminal outside VS Code)

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Starting Virtual Pet Nodes 🐾       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# Source workspace
cd ~/ROS2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# Check if Gazebo is running
if ! pgrep -x "gz" > /dev/null; then
    echo -e "${YELLOW}⚠ Warning: Gazebo doesn't seem to be running${NC}"
    echo -e "${YELLOW}Please start Gazebo first in another terminal:${NC}"
    echo "  ros2 launch turtlebot3_gazebo empty_world.launch.py"
    echo ""
    read -p "Continue anyway? [y/N]: " cont
    if [[ ! $cont =~ ^[Yy]$ ]]; then
        exit 0
    fi
fi

echo -e "${GREEN}✓ Ready to start pet nodes${NC}"
echo ""
echo -e "${YELLOW}Hand Gesture Controls:${NC}"
echo "  ✌️  Peace (2 fingers) → Print 'PACE!' & celebrate"
echo "  ✊  Fist → Draw Square"
echo "  ✋  Open Hand → Draw Circle"
echo "  👆  One Finger → Draw Line"
echo "  🤘  Rock Sign → Draw Triangle"
echo "  🤙  Three Fingers → Draw Star"
echo ""
echo -e "${GREEN}Starting nodes...${NC}"

ros2 launch virtual_pet pet_nodes_simple.launch.py
