# 🤖 Ollama LLM Integration Guide

## Overview

Pou now uses **Ollama** (local LLM) for natural, intelligent conversations! This makes Pou much more personable and able to understand context.

## Features

✅ **Natural Conversations** - Pou understands context and responds intelligently
✅ **Privacy First** - All processing happens locally on your machine
✅ **Memory** - Pou remembers recent conversation context
✅ **Softer Voice** - Slower, gentler speech (140 WPM, 85% volume)
✅ **Fallback Mode** - Still works without LLM using simple responses

## Quick Setup

### 1. Install Ollama (One-time setup):

```bash
cd ~/ROS2_ws
./setup_ollama.sh
```

This will:
- Install Ollama
- Pull the llama3.2:latest model (~2GB)
- Start the Ollama service

**OR manually:**
```bash
# Install
curl -fsSL https://ollama.com/install.sh | sh

# Start service
ollama serve

# Pull model (in another terminal)
ollama pull llama3.2:latest
```

### 2. Run the Virtual Pet System:

Same as before! Just launch normally:

**Terminal 1 - Gazebo:**
```bash
cd ~/ROS2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Terminal 2 - Pet System with LLM:**
```bash
cd ~/ROS2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch virtual_pet pet_complete.launch.py
```

## How It Works

### With LLM (Ollama Running):
- Pou uses AI to understand context
- Remembers last 5 conversation exchanges
- Generates natural, varied responses
- Personality: Friendly, playful virtual pet

### Without LLM (Fallback):
- Uses rule-based responses
- Still functional for commands
- No conversation memory

## Example Conversations

### Natural Conversations (with LLM):

**You:** "Hi friend, how are you today?"
**Pou:** "I'm doing wonderful! Just excited to hang out with you. What would you like to do together?"

**You:** "Tell me about yourself"
**Pou:** "Well, I'm Pou, your friendly virtual pet! I love drawing shapes and making you smile. Want me to draw something cool?"

**You:** "What's your favorite shape?"
**Pou:** "I really love stars! They're so sparkly and fun to draw. Would you like to see me draw one?"

**You:** "Draw a circle please"
**Pou:** *draws circle* "Okay! Watch me draw a circle! 🔵"

### Command Execution:

Commands still work instantly:
- "Circle" → Draws green circle
- "Square" → Draws red square  
- "Star" → Draws yellow star
- "Triangle" → Draws orange triangle
- "Line" → Draws blue line
- "Celebrate" → Spins and says "PACE!"

## Voice Settings

**Softer, More Natural:**
- Speed: 140 WPM (was 160)
- Volume: 85% (was 100%)
- Tone: Friendly and warm

## Model Options

### Recommended Models:

**llama3.2:latest** (Default)
- Size: ~2GB
- Speed: Fast
- Quality: Excellent for pet personality

**mistral:latest**
- Size: ~4GB  
- Speed: Fast
- Quality: Very intelligent

**phi3:latest**
- Size: ~2.3GB
- Speed: Very fast
- Quality: Good for simple conversations

### Change Model:

```bash
# Pull a different model
ollama pull mistral

# Edit voice_chat.py line 56:
# self.ollama_model = "mistral:latest"
```

## Troubleshooting

### "LLM not available" warning:

**Check if Ollama is running:**
```bash
curl http://localhost:11434/api/tags
```

**Start Ollama:**
```bash
ollama serve
```

**Check model is pulled:**
```bash
ollama list
```

**Pull model if missing:**
```bash
ollama pull llama3.2:latest
```

### Slow responses:

- First response is slower (model loading)
- Subsequent responses are fast
- Consider using smaller model: `phi3`

### High RAM usage:

- llama3.2: ~2-4GB RAM
- mistral: ~4-6GB RAM
- phi3: ~2-3GB RAM

## System Requirements

- **RAM:** 4GB minimum, 8GB recommended
- **Storage:** 3-5GB for model
- **CPU:** Modern x86_64 processor
- **GPU:** Optional (uses CPU by default)

## Benefits

### Compared to Cloud LLMs (ChatGPT, Claude):
✅ **Privacy** - Everything runs locally
✅ **Free** - No API costs
✅ **Fast** - No network latency
✅ **Offline** - Works without internet
✅ **Customizable** - Full control over model

### Compared to Rule-Based:
✅ **Natural** - Understands context
✅ **Varied** - Different responses each time
✅ **Smarter** - Can answer complex questions
✅ **Engaging** - Feels like real conversation

## Commands

```bash
# Check Ollama status
systemctl status ollama  # or: ps aux | grep ollama

# Stop Ollama
pkill ollama

# List installed models
ollama list

# Remove a model
ollama rm mistral

# Update Ollama
curl -fsSL https://ollama.com/install.sh | sh
```

---

**Enjoy natural conversations with Pou! 🤖💙**
