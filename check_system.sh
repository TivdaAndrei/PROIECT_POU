#!/bin/bash
# Quick System Check Script
# Run this if things aren't working to diagnose issues

echo "=========================================="
echo "Virtual Pet System Diagnostic Check"
echo "=========================================="
echo ""

# Check Ollama
echo "1. Checking Ollama LLM..."
if curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "   ✅ Ollama is running"
    echo "   Models installed:"
    curl -s http://localhost:11434/api/tags | grep -o '"name":"[^"]*"' | cut -d'"' -f4
else
    echo "   ❌ Ollama is NOT running"
    echo "   Fix: Run 'ollama serve' in another terminal"
fi
echo ""

# Check camera
echo "2. Checking camera devices..."
if ls /dev/video* > /dev/null 2>&1; then
    echo "   ✅ Camera devices found:"
    ls -l /dev/video* | awk '{print "      " $NF}'
else
    echo "   ❌ No camera devices found"
    echo "   Fix: Connect a webcam"
fi
echo ""

# Check ROS2 environment
echo "3. Checking ROS2 environment..."
if [ -n "$ROS_DISTRO" ]; then
    echo "   ✅ ROS2 environment: $ROS_DISTRO"
else
    echo "   ❌ ROS2 environment not sourced"
    echo "   Fix: source /opt/ros/jazzy/setup.bash"
fi
echo ""

# Check TurtleBot3 model
echo "4. Checking TurtleBot3 model..."
if [ -n "$TURTLEBOT3_MODEL" ]; then
    echo "   ✅ TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
else
    echo "   ❌ TURTLEBOT3_MODEL not set"
    echo "   Fix: export TURTLEBOT3_MODEL=burger"
fi
echo ""

# Check Python packages
echo "5. Checking Python dependencies..."
REQUIRED_PACKAGES=("mediapipe" "cv2" "numpy" "speech_recognition" "pyttsx3" "requests")
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if python3 -c "import $pkg" 2>/dev/null; then
        echo "   ✅ $pkg installed"
    else
        echo "   ❌ $pkg NOT installed"
        echo "   Fix: pip install $pkg"
    fi
done
echo ""

# Check Gazebo
echo "6. Checking Gazebo Harmonic..."
if command -v gz > /dev/null 2>&1; then
    echo "   ✅ Gazebo installed: $(gz sim --version 2>&1 | head -1)"
else
    echo "   ❌ Gazebo command 'gz' not found"
fi
echo ""

# Check ROS2 nodes (if system is running)
echo "7. Checking running ROS2 nodes..."
if ros2 node list 2>/dev/null | grep -q "gesture_recognizer"; then
    echo "   ✅ Virtual Pet system is RUNNING"
    echo "   Active nodes:"
    ros2 node list 2>/dev/null | grep -E "gesture|shape|pet|gui|voice" | sed 's/^/      /'
else
    echo "   ℹ️  Virtual Pet system not running (expected if you haven't launched yet)"
fi
echo ""

# Check topics (if system is running)
echo "8. Checking ROS2 topics..."
if ros2 topic list 2>/dev/null | grep -q "/pet/"; then
    echo "   ✅ Pet topics active:"
    ros2 topic list 2>/dev/null | grep "/pet/" | sed 's/^/      /'
else
    echo "   ℹ️  Pet topics not found (system not launched)"
fi
echo ""

# Check audio devices
echo "9. Checking audio output..."
if command -v pactl > /dev/null 2>&1; then
    if pactl list short sinks | grep -q "RUNNING\|IDLE"; then
        echo "   ✅ Audio output available"
    else
        echo "   ⚠️  No audio output detected"
    fi
else
    echo "   ℹ️  Cannot check audio (pactl not found)"
fi
echo ""

# Check microphone
echo "10. Checking microphone input..."
if command -v arecord > /dev/null 2>&1; then
    if arecord -l 2>/dev/null | grep -q "card"; then
        echo "   ✅ Microphone detected"
        arecord -l 2>/dev/null | grep "card" | sed 's/^/      /'
    else
        echo "   ❌ No microphone found"
        echo "   Fix: Connect a microphone for voice chat"
    fi
else
    echo "   ℹ️  Cannot check microphone (arecord not found)"
fi
echo ""

# Summary
echo "=========================================="
echo "Diagnostic Summary"
echo "=========================================="
echo ""

ISSUES=0

# Count issues
if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "⚠️  Ollama not running - voice chat will use fallback responses"
    ((ISSUES++))
fi

if ! ls /dev/video* > /dev/null 2>&1; then
    echo "❌ No camera - gesture control will NOT work"
    ((ISSUES++))
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced - system will NOT launch"
    ((ISSUES++))
fi

if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo "⚠️  TURTLEBOT3_MODEL not set - Gazebo may have issues"
    ((ISSUES++))
fi

if ! python3 -c "import mediapipe" 2>/dev/null; then
    echo "❌ mediapipe not installed - gesture recognition will FAIL"
    ((ISSUES++))
fi

if ! python3 -c "import speech_recognition" 2>/dev/null; then
    echo "❌ speech_recognition not installed - voice chat will FAIL"
    ((ISSUES++))
fi

if [ $ISSUES -eq 0 ]; then
    echo "✅ All systems ready! You can launch the virtual pet."
    echo ""
    echo "Next steps:"
    echo "  1. Terminal 1: ros2 launch turtlebot3_gazebo empty_world.launch.py"
    echo "  2. Terminal 2: ros2 launch virtual_pet pet_complete.launch.py"
    echo "  3. Say 'Hi friend' to start chatting!"
else
    echo ""
    echo "Found $ISSUES issue(s) - please fix them before launching"
fi

echo ""
echo "=========================================="
