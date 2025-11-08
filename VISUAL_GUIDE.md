# Quick Visual Guide - What to Expect

## When You Launch the System

After running `ros2 launch virtual_pet pet_complete.launch.py`, you'll see:

### 1. Three Windows Open Automatically ✅

```
┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐
│   Camera Window     │  │    Control GUI      │  │      RViz           │
│                     │  │                     │  │                     │
│  Your hands with    │  │  [Peace] [Fist]    │  │  TurtleBot3 +       │
│  green landmarks    │  │  [Open] [One]      │  │  Colorful trail     │
│  showing gesture    │  │  [Rock] [Three]    │  │  spheres            │
│  detection          │  │                     │  │                     │
│                     │  │  Alert Log:         │  │  (Red/Green/Blue/   │
│                     │  │  > INFO: Started    │  │   etc. spheres)     │
│                     │  │  > GESTURE: peace   │  │                     │
└─────────────────────┘  └─────────────────────┘  └─────────────────────┘
```

### 2. Terminal Output Shows

```bash
[INFO] [voice_chat]: 🤖 Voice Chat Node Started!
[INFO] [voice_chat]: ✅ Ollama LLM available - using AI conversations
[INFO] [voice_chat]: 🎤 Wake word: 'hi friend'
[INFO] [gesture_recognizer]: 📹 Camera initialized on /dev/video2
[INFO] [shape_drawer]: Shape Drawer Node Started!
[INFO] [gui_controller]: GUI Controller Started!
```

## How the System Responds

### Example 1: Natural Conversation

**You say:** "Hi friend, how are you feeling today?"

**What happens:**
1. Terminal shows: `[INFO] User said: "Hi friend, how are you feeling today?"`
2. GUI log shows: `CHAT_USER: Hi friend, how are you feeling today?` (blue text)
3. Ollama processes (~3 seconds)
4. Terminal shows: `[INFO] Pou: I'm so happy to see you! 🤗 Want to draw shapes together?`
5. GUI log shows: `CHAT_PET: I'm so happy to see you! 🤗` (pink text)
6. **Speakers say:** "I'm so happy to see you! Want to draw shapes together?"

### Example 2: Shape Command via Voice

**You say:** "Draw a circle please"

**What happens:**
1. Terminal shows: `[INFO] User said: "draw a circle please"`
2. GUI log shows: `COMMAND: Drawing circle` (yellow text)
3. **Speakers say:** "Okay! Watch me draw a circle! 🔵"
4. Robot starts moving in Gazebo (circular motion)
5. **Green spheres appear in RViz** following the robot's path
6. After ~8 seconds, robot stops

### Example 3: Hand Gesture

**You show:** ✌️ Peace sign to camera

**What happens:**
1. Camera window shows green landmarks on your hand
2. Terminal shows: `[INFO] [gesture_recognizer]: Detected: peace`
3. GUI log shows: `GESTURE: peace` (cyan text)
4. Robot spins in celebration in Gazebo
5. **Magenta/purple spheres appear in RViz**
6. Terminal shows: `✌️ PACE! Celebrating with a spin! 🎉`

### Example 4: GUI Button Click

**You click:** [Square] button

**What happens:**
1. GUI log shows: `COMMAND: Drawing square` (yellow text)
2. Robot draws square in Gazebo
3. **Red spheres appear in RViz** forming square shape
4. Takes about 12 seconds (4 sides + 4 turns)

## What You Should See in RViz

### Trail Visualization

```
    odom (world frame)
      ↓
    Robot path with colored spheres:

    Start → O O O O O → Turn
            (green)    ↓
                       O (green)
                       O (green)
                       O (green)
                     Turn
```

**Sphere properties:**
- Size: ~15cm diameter
- Height: Just above ground (7.5cm)
- Colors match gestures:
  - 🟢 Green = Circle (open hand)
  - 🔴 Red = Square (fist)
  - 🔵 Blue = Line (one finger)
  - 🟠 Orange = Triangle (rock)
  - 🟡 Yellow = Star (three fingers)
  - 🟣 Magenta = Peace/Celebration

**If you don't see spheres:**
1. Check RViz is showing the `/pet/trail_markers` topic
2. Fixed Frame should be "odom"
3. Robot must be MOVING to create trail
4. Try: `ros2 topic echo /pet/trail_markers` to verify publishing

## Voice Chat Behavior

### Wake Word Required
- First say: **"Hi friend"** (activates listening)
- Then you can talk normally

### What Triggers Shape Drawing
Any sentence containing these keywords:
- "**circle**" → Draws circle
- "**square**" → Draws square
- "**triangle**" → Draws triangle
- "**star**" → Draws star
- "**line**" → Draws line
- "**peace**" or "**celebrate**" → Celebration spin

### What Triggers Conversation
- Everything else! 
- Questions: "How are you?", "What's your name?"
- Statements: "I love you", "You're amazing"
- Any topic: "Tell me a joke", "What's your favorite color?"

## Troubleshooting Visual Guide

### Problem: No trail spheres in RViz
**Solution:**
1. Open RViz window
2. Click "Add" button (bottom left)
3. Select "By topic" tab
4. Find `/pet/trail_markers` → Marker
5. Click OK
6. Set Fixed Frame to "odom" (top left)

### Problem: Robot not moving
**Check terminal for:**
```
[WARN] No /cmd_vel subscribers
```
**Solution:** Robot not spawned in Gazebo. Run spawn command.

### Problem: No voice response
**Check terminal for:**
```
[WARN] Ollama not available - using fallback
```
**Solution:** 
```bash
ollama serve  # Start Ollama in another terminal
```

### Problem: Camera shows black screen
**Check terminal for:**
```
[ERROR] Could not open camera
```
**Solution:** Camera index wrong. Edit `gesture_recognizer.py` line with `VideoCapture(2)` to try 0 or 1.

## Success Indicators ✅

You know everything is working when:
- ✅ Camera shows your hands with green landmarks
- ✅ Terminal shows `Ollama LLM available`
- ✅ Speaking "Hi friend" gets a voice response
- ✅ Clicking GUI buttons makes robot move
- ✅ Colored spheres appear in RViz during movement
- ✅ Showing gestures triggers robot actions
- ✅ Pou responds naturally to your conversation

## Performance Expectations

- **LLM Response Time:** 2-7 seconds (first response slower as model loads)
- **Gesture Recognition:** <100ms (real-time)
- **Robot Response:** Immediate after command
- **Trail Spawn Rate:** Every 2cm of movement (~5-10 spheres per second during movement)
- **Voice Recognition:** 1-2 seconds after you stop speaking
- **GUI Button Click:** Instant response

## Fun Things to Try

1. **Voice conversation:**
   - "Hi friend" → "How are you?" → "Tell me a joke" → "You're so funny!"

2. **Mixed control:**
   - Say "Draw a circle" → While moving, show peace sign → Watch trail change colors!

3. **GUI rapid commands:**
   - Click [Circle] → wait → [Square] → wait → [Star] → Creates art!

4. **Natural language shapes:**
   - "Can you please show me how to draw a beautiful star?"
   - Pou understands and draws!

---

**Remember:** This is your virtual pet friend! Talk to it naturally, and it will respond like a real companion. The more you interact, the more fun you'll have! 🤖💙
