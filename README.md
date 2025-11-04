# PROIECT_ROS2_POU

## Virtual Pet Project 🐾

An interactive virtual pet using ROS2, Gazebo simulation, and hand gesture recognition!

### Features
- 🖐️ Control a TurtleBot3 robot using hand gestures
- 🎨 Draw shapes (square, circle, triangle, star, line) on the ground
- ✌️ Show peace sign to make the pet print "PACE!" and celebrate
- 🤖 Real-time computer vision with MediaPipe
- 🎮 Full Gazebo simulation environment

### Quick Start (RECOMMENDED METHOD)

**Terminal 1 - Start Gazebo:**
```bash
source ~/ROS2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Then manually add TurtleBot3 in Gazebo GUI (Insert tab → TurtleBot3 Burger)

**Terminal 2 - Start Pet Nodes:**
```bash
cd ~/ROS2_ws
./start_pet_nodes.sh
```

### Alternative: Use Helper Script
```bash
./start_virtual_pet_v2.sh
# Choose option 1, then in another terminal:
./start_pet_nodes.sh
```

### 🔧 Gazebo Issues?
See `GAZEBO_FIX_GUIDE.md` for troubleshooting help!

### Documentation
- 📖 Full README: `src/virtual_pet/README.md`
- 🔧 Fix Guide: `GAZEBO_FIX_GUIDE.md`
- 📋 Quick Ref: `QUICK_REFERENCE.txt`
- ✅ Setup: `SETUP_COMPLETE.md`
