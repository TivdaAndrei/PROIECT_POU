# Virtual Pet - Gazebo Crash Fix Guide 🔧

## The Problem

Gazebo GUI crashed with a segmentation fault. This is a known issue with Gazebo Harmonic and certain GPU/Qt configurations.

**Error seen:**
```
[gazebo-3] Segmentation fault (core dumped)
[ERROR] [gazebo-3]: process has died [pid 41602, exit code 139]
```

## ✅ SOLUTIONS (Choose One)

### **SOLUTION 1: Two-Step Startup (RECOMMENDED)**

This is the most reliable method:

#### Terminal 1 - Start Gazebo:
```bash
source ~/ROS2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Wait for Gazebo GUI to fully load, then manually add TurtleBot3:
- In Gazebo GUI, click the "Insert" tab (left panel)
- Search for "TurtleBot3 Burger"
- Click to add it to the world

#### Terminal 2 - Start Pet Nodes:
```bash
./start_pet_nodes.sh
```

---

### **SOLUTION 2: Use the Interactive Script**

```bash
./start_virtual_pet_v2.sh
```

Follow the menu:
- Choose option **1** (Empty world - most stable)
- Or option **3** if Gazebo is already running

---

### **SOLUTION 3: Headless Mode (No GUI)**

If you just want to test the nodes without visualization:

#### Terminal 1:
```bash
source ~/ROS2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
gz sim -r -s empty_world.sdf
```

#### Terminal 2:
```bash
./start_pet_nodes.sh
```

---

## 🎯 Quick Start Guide (Step by Step)

### **METHOD 1: Manual Startup (Most Reliable)**

1. **Open Terminal 1** - Start Gazebo:
   ```bash
   cd ~/ROS2_ws
   source install/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

2. **Wait** for Gazebo GUI to open completely (10-15 seconds)

3. **In Gazebo GUI**:
   - Look at the left panel
   - Click "Insert" tab
   - Scroll down to find "TurtleBot3 Burger"
   - Click on it, then click in the world to place it

4. **Open Terminal 2** - Start pet nodes:
   ```bash
   cd ~/ROS2_ws
   ./start_pet_nodes.sh
   ```

5. **Show hand gestures** to your webcam!

---

### **METHOD 2: Using Helper Scripts**

1. **Terminal 1**:
   ```bash
   cd ~/ROS2_ws
   ./start_virtual_pet_v2.sh
   ```
   Choose option **1**

2. After Gazebo opens, **Terminal 2**:
   ```bash
   cd ~/ROS2_ws
   ./start_pet_nodes.sh
   ```

---

## 🐛 Additional Troubleshooting

### If Gazebo keeps crashing:

1. **Update graphics drivers**:
   ```bash
   sudo apt update
   sudo apt upgrade
   ```

2. **Try software rendering**:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

3. **Reduce GUI complexity**:
   ```bash
   export GZ_GUI_RESOURCE_PATH=""
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

4. **Use older Gazebo world**:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
   ```

### If robot doesn't move:

1. Check Gazebo is not paused (press **Spacebar** in Gazebo)

2. Verify topics:
   ```bash
   ros2 topic list | grep cmd_vel
   ros2 topic echo /cmd_vel
   ```

3. Test manually:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```

### If camera doesn't work:

1. Test camera first:
   ```bash
   /home/andrei-robert/ROS2_ws/.venv/bin/python src/virtual_pet/test_camera.py
   ```

2. Check available cameras:
   ```bash
   ls -la /dev/video*
   ```

3. Try different camera index in `gesture_recognizer.py`:
   - Change line: `self.cap = cv2.VideoCapture(0)`
   - Try: `self.cap = cv2.VideoCapture(1)` or `2`

---

## 📊 Verify System Status

### Check what's running:
```bash
# Check ROS2 nodes
ros2 node list

# Check topics
ros2 topic list

# Check Gazebo
ps aux | grep gz
```

### Expected output:
```
/gesture_recognizer
/shape_drawer
/pet_controller
```

---

## 🎮 Testing Individual Components

### 1. Test Gesture Recognition Only:
```bash
source ~/ROS2_ws/install/setup.bash
ros2 run virtual_pet gesture_recognizer
```

### 2. Test in separate terminals:
```bash
# Terminal 1
ros2 run virtual_pet gesture_recognizer

# Terminal 2
ros2 run virtual_pet shape_drawer

# Terminal 3
ros2 topic echo /pet/gesture
```

---

## 🔄 Clean Restart

If everything is stuck:

```bash
# Kill all Gazebo and ROS processes
killall -9 gz gzserver gzclient ruby
pkill -9 -f ros2

# Clean and rebuild
cd ~/ROS2_ws
colcon build --packages-select virtual_pet --cmake-clean-cache

# Try again
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## ✨ Alternative: Test Without Gazebo

You can test gesture recognition without Gazebo:

```bash
# Terminal 1 - Just gesture recognizer
source ~/ROS2_ws/install/setup.bash
ros2 run virtual_pet gesture_recognizer

# Terminal 2 - Watch gestures being detected
ros2 topic echo /pet/gesture
```

This way you can verify the camera and gesture recognition work independently!

---

## 📝 Summary

**For beginners - Use this:**
1. Terminal 1: `ros2 launch turtlebot3_gazebo empty_world.launch.py`
2. Add robot manually in Gazebo GUI
3. Terminal 2: `./start_pet_nodes.sh`

**Quick option:**
```bash
./start_virtual_pet_v2.sh
# Choose option 1, then in another terminal:
./start_pet_nodes.sh
```

---

## 🆘 Still Having Issues?

Check the logs:
```bash
cat ~/.ros/log/latest/gazebo-3/stdout.log
```

Or test components individually as shown above!
