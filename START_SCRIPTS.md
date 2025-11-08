# 🚀 Quick Launch Scripts

Two convenient scripts to start your Virtual Pet system automatically!

## Option 1: Manual Robot Spawn (Recommended for first-time users)

```bash
./start_virtual_pet.sh
```

**What it does:**
- Opens Terminal 1: Gazebo Simulator (left side)
- Opens Terminal 2: Virtual Pet System (right side)
- Waits for you to spawn the robot manually

**After ~10 seconds, run this command:**
```bash
ros2 run gazebo_ros spawn_entity.py -entity turtlebot3 \
  -file ~/ROS2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x 0 -y 0 -z 0.01
```

**Use this when:** You want control over when the robot spawns.

---

## Option 2: Fully Automatic (One-Click Launch)

```bash
./start_complete.sh
```

**What it does:**
- Opens Terminal 1: Gazebo (left)
- Opens Terminal 2: Virtual Pet System (right)
- Opens Terminal 3: Auto-spawns robot (center, auto-closes)
- Everything happens automatically!

**Just wait ~20 seconds and you're ready!**

**Use this when:** You want the quickest startup.

---

## Comparison

| Feature | start_virtual_pet.sh | start_complete.sh |
|---------|---------------------|-------------------|
| Opens Gazebo | ✅ | ✅ |
| Opens Virtual Pet | ✅ | ✅ |
| Spawns Robot | ❌ (manual) | ✅ (automatic) |
| Startup Time | ~10s + manual | ~20s (fully auto) |
| Control | More control | Hands-free |
| Best For | First-time / debugging | Daily use |

---

## What Happens After Launch

Once everything is running, you'll see:

### Terminal 1 (Gazebo):
```
🌍 Gazebo Simulator
Starting simulation...
```

### Terminal 2 (Virtual Pet):
```
🤖 Virtual Pet System
[INFO] Gesture recognizer started
[INFO] Shape drawer started
[INFO] Voice chat started
[INFO] GUI started
📢 Say "Hi friend" to start chatting!
```

### Windows that Open:
- 🎮 **Gazebo** - 3D simulator with robot
- 📹 **Camera** - Your webcam for hand gestures
- 🖥️ **GUI** - Control buttons and alerts
- 🎨 **RViz** - Trail visualization

---

## How to Use After Launch

### 1. Voice Chat (Natural Language)
```
You: "Hi friend"
Pou: "Yes, I am listening. How may I assist you?"

You: "How are you?"
Pou: "I'm doing wonderful! Ready to have fun!"

You: "Draw a circle"
Pou: "Okay! Watch me draw a circle! 🔵"
[Robot draws in Gazebo, trails appear in RViz]
```

### 2. Hand Gestures
Show these to the camera:
- ✌️ Peace (V) → Celebration + magenta trail
- ✊ Fist → Square + red trail
- 🖐️ Open hand → Circle + green trail
- 👆 One finger → Line + blue trail
- 🤘 Rock → Triangle + orange trail
- 🤟 Three fingers → Star + yellow trail

### 3. GUI Buttons
Click buttons in the GUI window for instant commands.

---

## Stopping the System

### Quick Stop (Recommended):
Press `Ctrl+C` in Terminal 2 (Virtual Pet), then Terminal 1 (Gazebo).

### Force Kill:
```bash
killall -9 gz
killall -9 gzserver
killall -9 python3
```

---

## Troubleshooting

### "Command 'gnome-terminal' not found"
You're not using GNOME desktop. Edit the scripts to use your terminal:
- KDE: Replace `gnome-terminal` with `konsole -e`
- XFCE: Replace `gnome-terminal` with `xfce4-terminal -e`
- Others: Use `xterm -e` (basic)

### "Robot doesn't spawn automatically"
Use `start_virtual_pet.sh` instead and spawn manually. Check if Gazebo is fully loaded first.

### "Windows open in wrong positions"
Edit the script and adjust `--geometry=WIDTHxHEIGHT+X+Y` values.

### "ROS2 not found"
Make sure `source /opt/ros/jazzy/setup.bash` is in your `~/.bashrc`

---

## Customization

### Change Window Positions
Edit the `--geometry` parameters in the scripts:
```bash
--geometry=100x30+0+0      # 100 cols, 30 rows, at position (0,0)
--geometry=100x30+800+0    # Same size, at position (800,0)
```

### Change Startup Delays
Edit the `sleep` values:
```bash
sleep 3   # Wait 3 seconds before launching next terminal
sleep 10  # Wait 10 seconds for Gazebo to load
```

### Add More Terminals
Copy a terminal block and modify:
```bash
gnome-terminal --title="My Custom Terminal" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    # Your commands here
    exec bash
" &
```

---

## Advanced: Desktop Shortcut

Create a desktop launcher:

```bash
cat > ~/Desktop/virtual_pet.desktop << 'EOF'
[Desktop Entry]
Type=Application
Name=Virtual Pet
Comment=Launch Virtual Pet System
Exec=/home/YOUR_USERNAME/ROS2_ws/start_complete.sh
Icon=applications-robotics
Terminal=false
Categories=Robotics;Education;
EOF

chmod +x ~/Desktop/virtual_pet.desktop
```

Now you can double-click the desktop icon to launch!

---

## Tips

1. **First Time:** Use `start_virtual_pet.sh` to understand the process
2. **Daily Use:** Use `start_complete.sh` for fastest startup
3. **Debugging:** Manual spawn gives you more control
4. **Performance:** Close unused applications for better Gazebo performance
5. **Camera:** Make sure webcam isn't used by other apps

---

## Files Created

- `start_virtual_pet.sh` - Manual robot spawn version
- `start_complete.sh` - Fully automatic version
- `START_SCRIPTS.md` - This documentation

---

**Enjoy your Virtual Pet! 🤖💙**
