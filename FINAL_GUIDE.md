# 🎯 FINAL WORKING SOLUTION - Virtual Pet Startup Guide

## ✅ All Dependencies Fixed!
- NumPy 1.26.4 (compatible version)
- OpenCV 4.9.0 (compatible version)  
- MediaPipe 0.10.14
- gazebo_msgs installed

---

## 🚀 HOW TO RUN (Step-by-Step)

### **IMPORTANT: Use regular terminal, NOT VS Code terminal!**

Press `Ctrl+Alt+T` to open a system terminal (NOT from VS Code)

---

### **Terminal 1 - Start Gazebo:**

```bash
cd ~/ROS2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Wait for Gazebo GUI to open (10-15 seconds)**

Then **manually add the robot**:
1. Click **"Insert"** tab (left panel)
2. Find **"TurtleBot3 Burger"**
3. Click on it and place it in the world

**Keep this terminal running!**

---

### **Terminal 2 - Start Pet Nodes:**

Press `Ctrl+Alt+T` again to open a **SECOND** terminal

```bash
cd ~/ROS2_ws
./start_pet.sh
```

**You should see:**
- 3 nodes starting (gesture_recognizer, shape_drawer, pet_controller)
- OpenCV window opens showing camera feed
- Hand tracking overlay appears

---

## 🎮 Show These Gestures:

| Gesture | Action |
|---------|--------|
| ✌️ Peace (2 fingers) | Celebrates + spawns "PACE!" |
| ✊ Fist | Draws Square |
| ✋ Open Hand | Draws Circle |
| 👆 One Finger | Draws Line |
| 🤘 Rock | Draws Triangle |
| 🤙 Three Fingers | Draws Star |

---

## 🔍 Verify It's Working:

### Check if camera window opened:
- You should see an OpenCV window titled "Virtual Pet - Hand Gesture Recognition"
- Your camera feed should be visible
- When you show your hand, you should see hand landmarks drawn

### Check if nodes are running:
Open a **third** terminal:
```bash
source ~/ROS2_ws/install/setup.bash
ros2 node list
```

Should show:
```
/gesture_recognizer
/shape_drawer
/pet_controller
```

### Check if gestures are being detected:
```bash
ros2 topic echo /pet/gesture
```

Show a gesture to your camera - you should see messages like:
```
data: 'peace'
---
data: 'fist'
---
```

---

## 🐛 Troubleshooting:

### "No camera window appears"
- Check if camera is working: `ls -la /dev/video*`
- Test camera: `python3 ~/ROS2_ws/src/virtual_pet/test_camera.py`
- Make sure you're using a REGULAR terminal, not VS Code terminal

### "Robot doesn't move"
- Press **Spacebar** in Gazebo (might be paused)
- Check if robot is visible in Gazebo
- Test: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once`

### "Nodes fail to start"
- Make sure you're NOT in VS Code terminal (it has venv active)
- Use `Ctrl+Alt+T` to open system terminal
- Check Python: `/usr/bin/python3 -c "import cv2, mediapipe; print('OK')"`

### "Import errors"
- Packages are installed to system Python at: `~/.local/lib/python3.12/site-packages/`
- VS Code terminal uses venv, system terminal doesn't

---

## 📋 Quick Command Reference:

```bash
# Start Gazebo (Terminal 1)
cd ~/ROS2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Start Pet (Terminal 2 - REGULAR terminal, not VS Code!)
cd ~/ROS2_ws
./start_pet.sh

# Monitor gestures (Terminal 3 - optional)
source ~/ROS2_ws/install/setup.bash
ros2 topic echo /pet/gesture
```

---

## ⚠️ KEY POINT:

**DO NOT use VS Code integrated terminal!**  
Use system terminal (`Ctrl+Alt+T`)

This is because VS Code automatically activates the virtual environment (.venv) which has different Python packages.

---

## 🎉 You're Ready!

Once both terminals are running and the camera window is open, just show gestures to your camera and watch the TurtleBot3 draw shapes!

Have fun with your virtual pet! 🐾✨
