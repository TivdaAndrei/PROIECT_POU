#!/bin/bash
# Test Virtual Pet Nodes Without Gazebo
# This verifies gesture recognition and shape logic work independently

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}╔═══════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Virtual Pet - Component Test (No Gazebo)║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════╝${NC}"
echo ""

# Source workspace
source /home/andrei-robert/ROS2_ws/install/setup.bash

echo -e "${YELLOW}This test runs gesture recognition without Gazebo${NC}"
echo -e "${YELLOW}Great for verifying camera and gesture detection!${NC}"
echo ""

# Test 1: Camera
echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}TEST 1: Camera Check${NC}"
echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo ""
echo "Testing camera... Press 'q' to continue"
/home/andrei-robert/ROS2_ws/.venv/bin/python /home/andrei-robert/ROS2_ws/src/virtual_pet/test_camera.py

# Test 2: Gesture Recognizer
echo ""
echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo -e "${GREEN}TEST 2: Gesture Recognition${NC}"
echo -e "${BLUE}═══════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Starting gesture recognizer node...${NC}"
echo "Try different hand gestures!"
echo ""
echo "In another terminal, run:"
echo -e "${GREEN}  ros2 topic echo /pet/gesture${NC}"
echo ""
echo "To see detected gestures in real-time!"
echo ""
echo -e "${RED}Press Ctrl+C to stop${NC}"
echo ""
sleep 3

ros2 run virtual_pet gesture_recognizer
