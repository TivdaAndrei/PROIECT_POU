# Virtual Pet - ROS2 Interactive Robot with Voice Chat

A ROS2 project featuring an intelligent virtual pet (TurtleBot3) that responds to hand gestures, voice commands, and natural conversation. The robot draws colorful shapes, leaves visual trails, and chats with you like a friend!

## Features

- 🤖 **Hand Gesture Control** - Control robot using MediaPipe hand tracking
- � **Voice Chat** - Talk naturally with your pet using Ollama LLM
- 🗣️ **Natural Conversation** - Pou responds to ANY topic like a real friend
- �🎨 **Colorful Trails** - Robot leaves colored spheres as it moves (visible in RViz)
- 🖥️ **GUI Interface** - Control buttons and real-time alerts
- 🔊 **Text-to-Speech** - Pou talks back with a friendly voice
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
- Ollama (local LLM for conversation)
- MediaPipe, OpenCV, NumPy, SpeechRecognition, pyttsx3 (installed to system Python)

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
- 🎨 **RViz** - Colored trail visualization (spheres appear as robot moves!)
- 🎤 **Voice Chat** - Microphone listening for commands and conversation

## How to Interact with Pou

### 1. Voice Chat (Natural Conversation)
**Wake word:** "Hi friend"

Then talk naturally about ANYTHING:
- "How are you feeling today?"
- "Tell me a joke"
- "I love you"
- "What's your favorite color?"
- "I'm feeling sad"
- "You're amazing"

**Pou will respond naturally to everything!** 💬

### 2. Voice Commands (Shape Drawing)
Say these keywords in any sentence:
- "Draw a **circle**" → Green circular trail
- "Make a **square**" → Red square trail
- "Show me a **triangle**" → Orange triangle trail
- "Can you draw a **star**?" → Yellow star trail
- "**Line** please" → Blue straight line
- "**Peace**!" → Magenta celebration spin

### 3. Hand Gestures
Show these gestures to the camera:
- ✌️ Peace sign → Celebration + "PACE!"
- ✊ Fist → Square
- 🖐️ Open hand → Circle
- 👆 One finger → Line
- 🤘 Rock sign → Triangle
- 🤟 Three fingers → Star

### 4. GUI Buttons
Click buttons in the GUI window for instant shape commands.

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
