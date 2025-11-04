#!/bin/bash
# Quick start script for Virtual Pet project

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     Virtual Pet - Quick Start 🐾      ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# Source ROS2 workspace
echo -e "${YELLOW}Sourcing ROS2 workspace...${NC}"
source /home/andrei-robert/ROS2_ws/install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger
echo -e "${GREEN}✓ TurtleBot3 model set to: $TURTLEBOT3_MODEL${NC}"

# Check if Python packages are installed
echo -e "${YELLOW}Checking Python dependencies...${NC}"
/home/andrei-robert/ROS2_ws/.venv/bin/python -c "import cv2, mediapipe" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Python dependencies OK${NC}"
else
    echo -e "${YELLOW}⚠ Installing Python dependencies...${NC}"
    /home/andrei-robert/ROS2_ws/.venv/bin/pip install -r /home/andrei-robert/ROS2_ws/src/virtual_pet/requirements.txt
fi

echo ""
echo -e "${BLUE}Starting Virtual Pet...${NC}"
echo -e "${YELLOW}Hand Gesture Controls:${NC}"
echo "  ✌️  Peace (2 fingers) → Print 'PACE!' & celebrate"
echo "  ✊  Fist → Draw Square"
echo "  ✋  Open Hand → Draw Circle"
echo "  👆  One Finger → Draw Line"
echo "  🤘  Rock Sign → Draw Triangle"
echo "  🤙  Three Fingers → Draw Star"
echo ""
echo -e "${GREEN}Launching in 3 seconds...${NC}"
sleep 3

# Launch the virtual pet
ros2 launch virtual_pet virtual_pet.launch.py
