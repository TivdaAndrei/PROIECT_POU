# 🎯 Virtual Pet - Working Solution

## ⚠️ Gazebo Crash Issue - SOLVED!

The original launch crashed due to Gazebo GUI rendering issues. Here's how to run the project successfully:

---

## ✅ WORKING METHOD (Choose One)

### **METHOD 1: Two-Terminal Approach (MOST RELIABLE)**

This separates Gazebo from the pet nodes for maximum stability.

#### 📺 Terminal 1 - Start Gazebo:
```bash
cd ~/ROS2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Wait for Gazebo GUI to open, then:**
- Click **"Insert"** tab (left panel in Gazebo)
- Find **"TurtleBot3 Burger"**
- Click on it and place it in the world

#### 🐾 Terminal 2 - Start Pet Nodes:
```bash
cd ~/ROS2_ws
./start_pet_nodes.sh
```

**That's it!** Show hand gestures to your camera!

---

### **METHOD 2: Interactive Menu Script**

```bash
cd ~/ROS2_ws
./start_virtual_pet_v2.sh
```

Follow the menu and choose option **1** (Empty World).

Then in another terminal:
```bash
cd ~/ROS2_ws
./start_pet_nodes.sh
```

---

### **METHOD 3: Test Without Gazebo**

Just want to test gesture recognition?

```bash
cd ~/ROS2_ws
./test_virtual_pet.sh
```

This runs the gesture recognizer independently!

---

## 🎮 Gesture Controls

Once running, show these gestures to your webcam:

| Gesture | What Happens |
|---------|--------------|
| ✌️ **Peace Sign** (2 fingers) | Robot spins & spawns "PACE!" text |
| ✊ **Fist** (closed hand) | Draws a **Square** |
| ✋ **Open Hand** (5 fingers) | Draws a **Circle** |
| 👆 **One Finger** (pointing up) | Draws a **Line** |
| 🤘 **Rock Sign** (thumb + pinky) | Draws a **Triangle** |
| 🤙 **Three Fingers** | Draws a **Star** |

---

## 📋 Available Scripts

All in `~/ROS2_ws/`:

| Script | Purpose |
|--------|---------|
| `start_virtual_pet_v2.sh` | Interactive menu launcher |
| `start_pet_nodes.sh` | Start pet nodes only |
| `test_virtual_pet.sh` | Test without Gazebo |
| `test_camera.py` | Test webcam only |

---

## 🔍 Quick Diagnostics

### Check if everything is running:
```bash
# Should show 3 nodes
ros2 node list

# Should show /pet/gesture topic
ros2 topic list | grep pet

# Watch for detected gestures
ros2 topic echo /pet/gesture
```

### Test robot control manually:
```bash
# Make robot move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# Make robot rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once
```

---

## 🐛 Common Issues & Solutions

### Issue: "Camera not found"
**Solution:**
```bash
# Check cameras
ls -la /dev/video*

# Test camera
/home/andrei-robert/ROS2_ws/.venv/bin/python src/virtual_pet/test_camera.py
```

### Issue: "Robot doesn't move"
**Solution:**
1. Check if Gazebo is **paused** (press Spacebar to unpause)
2. Verify robot is spawned in Gazebo
3. Test: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once`

### Issue: "No gestures detected"
**Solution:**
1. Ensure good lighting
2. Position hand 0.5-1.5m from camera
3. Make clear, distinct gestures
4. Check OpenCV window for visual feedback

### Issue: "Gazebo crashes immediately"
**Solution:**
```bash
# Use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

---

## 📊 System Architecture

```
┌─────────────┐
│   Webcam    │
└──────┬──────┘
       │
       ▼
┌─────────────────────┐
│ gesture_recognizer  │ ─── publishes to ───▶ /pet/gesture
└─────────────────────┘
       │
       ▼
┌──────────────────┐
│  shape_drawer    │ ─── publishes to ───▶ /cmd_vel
└──────────────────┘
       │
       ▼
┌──────────────────┐
│  TurtleBot3      │
│  (in Gazebo)     │
└──────────────────┘
```

---

## 📚 Documentation Files

- **GAZEBO_FIX_GUIDE.md** - Detailed troubleshooting
- **QUICK_REFERENCE.txt** - Command reference card
- **SETUP_COMPLETE.md** - Full setup documentation
- **src/virtual_pet/README.md** - Package documentation
- **src/virtual_pet/ARCHITECTURE.py** - System diagram

---

## 🎯 Testing Checklist

- [ ] Camera works (`test_camera.py`)
- [ ] Gazebo launches without crashing
- [ ] TurtleBot3 is visible in Gazebo
- [ ] Pet nodes start successfully
- [ ] Gestures are detected (check OpenCV window)
- [ ] Robot responds to gestures
- [ ] Peace sign triggers celebration

---

## 🚀 Full Example Session

```bash
# Terminal 1 - Start Gazebo
cd ~/ROS2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
# Wait for GUI, then insert TurtleBot3 manually

# Terminal 2 - Start Pet
cd ~/ROS2_ws
./start_pet_nodes.sh
# Show gestures to camera!

# Terminal 3 - Monitor (optional)
ros2 topic echo /pet/gesture
```

---

## 💡 Pro Tips

1. **Best lighting**: Bright, even light on your hand
2. **Camera position**: Face camera directly, 0.5-1.5m away
3. **Gesture clarity**: Make distinct, clear gestures
4. **Cooldown**: Wait 1.5 seconds between gestures
5. **Gazebo view**: Use Gazebo camera controls to see robot better
6. **Performance**: Close other heavy applications

---

## 🎊 What's Working

✅ Hand gesture recognition (MediaPipe + OpenCV)  
✅ ROS2 node communication  
✅ Robot movement control  
✅ Shape drawing algorithms  
✅ Peace sign celebration  
✅ System monitoring  
✅ Stable operation (with proper launch method)  

---

## 📞 Need More Help?

1. Read: `GAZEBO_FIX_GUIDE.md`
2. Check: `QUICK_REFERENCE.txt`
3. Test components individually: `./test_virtual_pet.sh`
4. Monitor topics: `ros2 topic list` and `ros2 topic echo`

---

## 🎉 Have Fun!

Your virtual pet is ready to play! Show it different gestures and watch it draw shapes in Gazebo! ✨🐾

**Remember:** Use the two-terminal method for best results!
