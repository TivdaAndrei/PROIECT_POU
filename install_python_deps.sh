#!/bin/bash
# Install Python packages to system (not venv) for ROS2

echo "Installing Python packages to SYSTEM Python (not venv)..."
echo "This is required for ROS2 nodes to work properly"
echo ""

# Use absolute path to system Python to bypass venv
/usr/bin/python3 -m pip install --break-system-packages opencv-python mediapipe numpy

echo ""
echo "✓ Installation complete!"
echo ""
echo "Now rebuild the package:"
echo "  cd ~/ROS2_ws"
echo "  colcon build --packages-select virtual_pet --symlink-install"
