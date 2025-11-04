# Virtual Pet - ROS2 & Gazebo Project 🐾

A fun interactive virtual pet project using ROS2, Gazebo, and computer vision! Control a TurtleBot3 robot to draw shapes on the ground using hand gestures, and make it celebrate with a "PACE!" message when you show a peace sign.

## Features ✨

- 🖐️ **Hand Gesture Recognition** - Uses MediaPipe and OpenCV to detect hand gestures in real-time
- 🎨 **Shape Drawing** - Virtual pet draws different shapes based on your gestures
- 🤖 **TurtleBot3 Integration** - Simulated robot in Gazebo environment
- 🎉 **Interactive Feedback** - Pet celebrates and spawns text when you show specific gestures
- 🐍 **Python Implementation** - Fully implemented in Python for easy understanding

## Gesture Controls 👋

| Gesture | Description | Action |
|---------|-------------|--------|
| ✌️ Peace Sign (2 fingers) | Index + Middle finger extended | Prints "PACE!" text and celebrates with a spin! |
| ✊ Fist | All fingers closed | Draws a **Square** |
| ✋ Open Hand (5 fingers) | All fingers extended | Draws a **Circle** |
| 👆 One Finger | Only index finger extended | Draws a **Line** |
| 🤘 Rock Sign | Thumb + Pinky extended | Draws a **Triangle** |
| 🤙 Three Fingers | Three fingers extended | Draws a **Star** |

## Architecture 🏗️

The project consists of three main ROS2 nodes:

1. **Gesture Recognizer Node** (`gesture_recognizer.py`)
   - Captures video from webcam
   - Uses MediaPipe for hand tracking
   - Recognizes gestures and publishes to `/pet/gesture` topic

2. **Shape Drawer Node** (`shape_drawer.py`)
   - Subscribes to `/pet/gesture` topic
   - Calculates movement patterns for shapes
   - Publishes velocity commands to `/cmd_vel` topic
   - Spawns "PACE!" text in Gazebo for peace sign

3. **Pet Controller Node** (`pet_controller.py`)
   - Monitors system status
   - Tracks gesture statistics
   - Publishes status updates

## Prerequisites 📋

- ROS2 (Humble or newer)
- Python 3.8+
- TurtleBot3 packages
- Gazebo
- Webcam

## Installation 🚀

### 1. Install Python Dependencies

```bash
cd ~/ROS2_ws/src/virtual_pet
pip install -r requirements.txt
```

### 2. Set TurtleBot3 Model

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 3. Build the Workspace

```bash
cd ~/ROS2_ws
colcon build --packages-select virtual_pet
source install/setup.bash
```

## Usage 🎮

### Quick Start (All-in-One Launch)

Launch everything at once (Gazebo + Virtual Pet nodes):

```bash
source ~/ROS2_ws/install/setup.bash
ros2 launch virtual_pet virtual_pet.launch.py
```

### Alternative: Manual Launch

**Terminal 1 - Start Gazebo with TurtleBot3:**
```bash
source ~/ROS2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Start Virtual Pet Nodes:**
```bash
source ~/ROS2_ws/install/setup.bash
ros2 launch virtual_pet pet_nodes_only.launch.py
```

### Running Individual Nodes

For debugging or testing:

```bash
# Gesture recognizer only
ros2 run virtual_pet gesture_recognizer

# Shape drawer only
ros2 run virtual_pet shape_drawer

# Pet controller only
ros2 run virtual_pet pet_controller
```

## How It Works 🔧

1. **Camera Capture**: The gesture recognizer node captures video from your webcam
2. **Hand Detection**: MediaPipe detects hand landmarks in real-time
3. **Gesture Recognition**: Finger positions are analyzed to determine the gesture
4. **ROS2 Communication**: Recognized gesture is published to `/pet/gesture` topic
5. **Shape Drawing**: Shape drawer node receives gesture and calculates movement pattern
6. **Robot Control**: Velocity commands are sent to TurtleBot3 via `/cmd_vel` topic
7. **Visual Feedback**: Robot draws shapes in Gazebo, or spawns text for peace sign

## ROS2 Topics 📡

- `/pet/gesture` (std_msgs/String) - Recognized gestures
- `/pet/status` (std_msgs/String) - System status updates
- `/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands

## ROS2 Services 🔧

- `/spawn_entity` (gazebo_msgs/SpawnEntity) - Spawn objects in Gazebo

## Troubleshooting 🔍

### Camera Not Working
- Check if webcam is connected: `ls /dev/video*`
- Try different camera index in `gesture_recognizer.py`: `cv2.VideoCapture(1)`

### Gazebo Not Starting
- Check if TurtleBot3 model is set: `echo $TURTLEBOT3_MODEL`
- Ensure TurtleBot3 packages are installed: `ros2 pkg list | grep turtlebot3`

### Robot Not Moving
- Verify topic connection: `ros2 topic echo /cmd_vel`
- Check if Gazebo is paused (press spacebar in Gazebo to unpause)

### Gestures Not Detected
- Ensure good lighting conditions
- Position hand clearly in front of camera
- Check gesture recognizer window for visual feedback

## Customization 🎨

### Adjust Drawing Speed
Edit `shape_drawer.py`:
```python
self.linear_speed = 0.2  # Increase for faster drawing
self.angular_speed = 0.5  # Increase for sharper turns
```

### Add New Gestures
1. Add gesture detection logic in `gesture_recognizer.py`
2. Add corresponding shape drawing method in `shape_drawer.py`
3. Map gesture to shape in `gesture_callback()`

### Change Shape Sizes
Edit `shape_drawer.py`:
```python
self.side_length = 1.0  # Increase for larger shapes
```

## Project Structure 📁

```
virtual_pet/
├── launch/
│   ├── virtual_pet.launch.py          # Full system launch
│   └── pet_nodes_only.launch.py       # Pet nodes only
├── virtual_pet/
│   ├── __init__.py
│   ├── gesture_recognizer.py          # Hand gesture detection
│   ├── shape_drawer.py                # Shape drawing controller
│   └── pet_controller.py              # System coordinator
├── resource/
│   └── virtual_pet
├── package.xml                         # ROS2 package manifest
├── setup.py                            # Python package setup
├── setup.cfg                           # Python package config
├── requirements.txt                    # Python dependencies
└── README.md                           # This file
```

## Future Enhancements 🚀

- [ ] Add trail visualization in Gazebo to show drawn paths
- [ ] Implement voice feedback using text-to-speech
- [ ] Add more complex shapes (heart, flower, etc.)
- [ ] Create gesture for "undo" last shape
- [ ] Add gesture chaining for complex patterns
- [ ] Integrate with RViz2 for better visualization
- [ ] Add gesture for robot speed control
- [ ] Implement multi-robot synchronized drawing

## Credits 👏

- Built with ROS2 and Gazebo
- Uses [MediaPipe](https://google.github.io/mediapipe/) for hand tracking
- TurtleBot3 by ROBOTIS

## License 📄

Apache-2.0

## Contributing 🤝

Feel free to fork, improve, and submit pull requests! This is a learning project and contributions are welcome.

---

**Have fun playing with your virtual pet! 🐾✨**
