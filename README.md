# Virtual Pet - ROS2 Hand Gesture Control

A ROS2 project that controls a TurtleBot3 robot in Gazebo using hand gestures. The robot draws colorful shapes and leaves visual trails.

## Features

- 🤖 **Hand Gesture Control** - Control robot using MediaPipe hand tracking
- 🎨 **Colorful Trails** - Robot leaves colored spheres as it moves
- 🖥️ **GUI Interface** - Control buttons and real-time alerts
- ✌️ **6 Gestures**:
  - Peace (V) → Magenta trail + celebration spin + "PACE!"
  - Fist → Red square
  - Open hand → Green circle
  - One finger → Blue line
  - Rock sign → Orange triangle
  - Three fingers → Yellow star

## Requirements

- ROS2 Jazzy
- Python 3.12
- Gazebo Harmonic
- TurtleBot3 packages
- MediaPipe, OpenCV, NumPy (installed to system Python)

## Quick Start

### Terminal 1 - Launch Gazebo:
```bash
cd ~/ROS2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Terminal 2 - Launch Virtual Pet System:
```bash
cd ~/ROS2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch virtual_pet pet_complete.launch.py
```

This will open:
- 🖥️ **GUI** - Control buttons and alerts
- 📹 **Camera** - Hand gesture detection
- 🎨 **RViz** - Colored trail visualization

## RViz Setup (First Time)

If trails don't appear in RViz:
1. Click **"Add"** → **"By topic"**
2. Select `/pet/trail_markers` → **Marker**
3. Change **"Fixed Frame"** to **`odom`**

## Project Structure

```
src/virtual_pet/
├── virtual_pet/
│   ├── gesture_recognizer.py   # Hand gesture detection
│   ├── shape_drawer.py          # Robot movement & trail drawing
│   ├── pet_controller.py        # System monitoring
│   └── gui_controller.py        # GUI interface
├── launch/
│   └── pet_complete.launch.py  # Main launch file
├── rviz/
│   └── trail_view.rviz         # RViz configuration
└── package.xml
```

## Troubleshooting

- **No camera window**: Check camera at `/dev/video2`
- **No trails in RViz**: Verify Fixed Frame is set to `odom`
- **Robot doesn't move**: Ensure Gazebo is fully loaded before starting pet system
- **Import errors**: Use system terminal, not VS Code integrated terminal

## Controls

- Use hand gestures in front of camera
- OR click buttons in GUI window
- Watch real-time alerts in GUI (not terminal)
