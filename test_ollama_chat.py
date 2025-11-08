#!/usr/bin/env python3
"""Test Ollama conversation responses"""

import requests
import json

ollama_url = "http://localhost:11434/api/generate"
model = "llama3.2:latest"

personality = """You are Pou, a friendly and playful virtual pet robot. 
You are cheerful, curious, and love to draw shapes like circles, squares, triangles, stars, and lines.
You're warm and caring, like a best friend. Keep responses SHORT (1-2 sentences max) and very friendly.
Use emojis occasionally. Be enthusiastic about drawing shapes!"""

def test_conversation(user_input):
    """Test a conversation with Ollama"""
    context = f"{personality}\n\nUser: {user_input}\nPou:"
    
    print(f"\n👤 User: {user_input}")
    print("🤖 Pou is thinking...")
    
    try:
        response = requests.post(
            ollama_url,
            json={
                "model": model,
                "prompt": context,
                "stream": False,
                "options": {
                    "temperature": 0.7,
                    "top_p": 0.9,
                    "num_predict": 50
                }
            },
            timeout=10
        )
        
        if response.status_code == 200:
            result = response.json()
            ai_response = result.get('response', '').strip()
            
            # Clean up
            lines = [line.strip() for line in ai_response.split('\n') if line.strip()]
            clean_response = lines[0] if lines else "I'm happy!"
            
            print(f"🤖 Pou: {clean_response}")
            return clean_response
        else:
            print(f"❌ Error: {response.status_code}")
            return None
            
    except Exception as e:
        print(f"❌ Failed: {e}")
        return None

if __name__ == "__main__":
    print("=" * 60)
    print("Testing Ollama Conversation")
    print("=" * 60)
    
    # Test various inputs
    test_cases = [
        "Hi friend",
        "How are you?",
        "I love you",
        "You're amazing",
        "Tell me a joke",
        "What's your favorite color?",
        "I'm feeling sad today"
    ]
    
    for test in test_cases:
        test_conversation(test)
    
    print("\n" + "=" * 60)
    print("Test complete!")
    print("=" * 60)
