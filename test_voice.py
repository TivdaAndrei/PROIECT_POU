#!/usr/bin/env python3
"""Test if text-to-speech works"""

import pyttsx3

print("🔊 Testing Text-to-Speech...")
print("")

try:
    # Initialize TTS
    engine = pyttsx3.init()
    
    # Get available voices
    voices = engine.getProperty('voices')
    print(f"✅ Found {len(voices)} voices:")
    for i, voice in enumerate(voices):
        print(f"  [{i}] {voice.name} - {voice.id}")
    print("")
    
    # Configure voice
    engine.setProperty('rate', 160)
    engine.setProperty('volume', 1.0)
    
    # Try male voice
    selected = 28  # English (America) - index from list above
    for i, voice in enumerate(voices):
        if 'en-us' in voice.id.lower():
            selected = i
            break
    
    engine.setProperty('voice', voices[selected].id)
    
    print(f"🎤 Using voice: {voices[selected].name}")
    print("🔊 Speaking: 'Hello, I am Pou, your virtual assistant. Say hi friend to activate me.'")
    print("")
    
    # Speak
    engine.say("Hello, I am Pou, your virtual assistant. Say hi friend to activate me.")
    engine.runAndWait()
    
    print("✅ TTS working! You should have heard speech from your speakers.")
    print("")
    print("If you didn't hear anything:")
    print("1. Check speaker volume")
    print("2. Install espeak: sudo apt install espeak espeak-data")
    print("3. Test speakers: speaker-test -t wav -c 2")
    
except Exception as e:
    print(f"❌ Error: {e}")
    print("")
    print("Fix:")
    print("  sudo apt install espeak espeak-data libespeak-dev")
    print("  pip3 install --break-system-packages pyttsx3 --force-reinstall")
