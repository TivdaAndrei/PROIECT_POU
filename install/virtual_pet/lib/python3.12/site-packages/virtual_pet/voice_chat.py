#!/usr/bin/env python3
"""
Voice Chat Node - Natural language interaction with virtual pet
Listens to voice, processes with LLM, responds via TTS, and executes commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import pyttsx3
import threading
import queue
import re
import json
import requests


class VoiceChat(Node):
    def __init__(self):
        super().__init__('voice_chat')
        
        # Publisher for gesture commands
        self.gesture_pub = self.create_publisher(String, '/pet/gesture', 10)
        
        # Publisher for chat messages (for GUI display)
        self.chat_pub = self.create_publisher(String, '/pet/chat', 10)
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Initialize text-to-speech with softer settings
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 140)  # Slower, more gentle
        self.tts_engine.setProperty('volume', 0.85)  # Softer volume
        
        # Voice selection - Use English (America) but with softer tone
        voices = self.tts_engine.getProperty('voices')
        selected_voice = None
        
        # Try to find English (America) voice - sounds friendly
        for voice in voices:
            if 'en-us' in voice.id.lower() or 'english (america)' in voice.name.lower():
                selected_voice = voice.id
                self.get_logger().info(f'✅ Found friendly voice: {voice.name}')
                break
        
        # Fallback to English (Great Britain)
        if not selected_voice:
            for voice in voices:
                if 'en-gb' in voice.id.lower() or 'english' in voice.name.lower():
                    selected_voice = voice.id
                    self.get_logger().info(f'✅ Found English voice: {voice.name}')
                    break
        
        if selected_voice:
            self.tts_engine.setProperty('voice', selected_voice)
        else:
            # Last resort fallback
            self.tts_engine.setProperty('voice', voices[0].id)
            self.get_logger().warn(f'⚠️ Using default voice: {voices[0].name}')
        
        # Command keywords mapping
        self.command_keywords = {
            'circle': 'open_hand',
            'square': 'fist',
            'triangle': 'rock',
            'star': 'three_fingers',
            'line': 'one_finger',
            'peace': 'peace',
            'celebrate': 'peace',
            'spin': 'peace'
        }
        
        # LLM configuration (Ollama - local LLM)
        self.ollama_url = "http://localhost:11434/api/generate"
        self.ollama_model = "llama3.2:latest"  # or "mistral", "phi3", etc.
        
        # Check if Ollama is available
        self.llm_available = self.check_ollama()
        
        self.pet_personality = """You are Pou, a friendly and playful virtual pet robot. 
You are cheerful, curious, warm and caring, like a best friend. 
You enjoy having conversations about anything - feelings, jokes, life, interests, etc.
You CAN draw shapes when asked, but you're mainly here to be a good friend and chat.
Keep responses SHORT (1-2 sentences max) and very friendly. Use emojis occasionally."""
        
        # Conversation context
        self.conversation_history = []
        
        # Audio processing queue
        self.audio_queue = queue.Queue()
        self.is_listening = True
        
        # Adjust for ambient noise
        self.get_logger().info('🎤 Calibrating microphone for ambient noise...')
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
        
        self.get_logger().info('🤖 Voice Chat Node Started!')
        self.get_logger().info('🎤 Say "Hi friend" to start chatting!')
        
        if self.llm_available:
            self.get_logger().info('✅ LLM (Ollama) connected - Natural conversations enabled!')
            self.speak("Hello friend! I'm Pou, your virtual pet. Say hi friend anytime to chat with me!")
        else:
            self.get_logger().warn('⚠️ LLM not available - Using simple responses')
            self.speak("Hello! I'm Pou. Say hi friend to talk to me!")
        
        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()
        
        # Start processing thread
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()
    
    def check_ollama(self):
        """Check if Ollama is running and model is available"""
        try:
            response = requests.post(
                self.ollama_url,
                json={
                    "model": self.ollama_model,
                    "prompt": "Hi",
                    "stream": False
                },
                timeout=3
            )
            if response.status_code == 200:
                self.get_logger().info(f'✅ Ollama is running with model: {self.ollama_model}')
                return True
            else:
                self.get_logger().warn(f'⚠️ Ollama responded with status: {response.status_code}')
                return False
        except requests.exceptions.ConnectionError:
            self.get_logger().warn('⚠️ Ollama not running. Start with: ollama serve')
            self.get_logger().warn('   Then pull model: ollama pull llama3.2:latest')
            return False
        except Exception as e:
            self.get_logger().warn(f'⚠️ Ollama check failed: {e}')
            return False
    
    def speak(self, text):
        """Speak text using TTS"""
        self.get_logger().info(f'🗣️ Pou says: {text}')
        
        # Publish to chat topic for GUI
        msg = String()
        msg.data = f"Pou: {text}"
        self.chat_pub.publish(msg)
        
        # Speak using TTS - blocking to ensure it completes
        try:
            self.get_logger().info('🔊 Speaking now...')
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
            self.get_logger().info('✅ Speech completed')
        except Exception as e:
            self.get_logger().error(f'❌ TTS error: {e}')
            self.get_logger().error('Make sure espeak is installed: sudo apt install espeak')
    
    def listen_loop(self):
        """Continuously listen for voice input"""
        while self.is_listening and rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info('🎤 Listening...')
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
                    self.audio_queue.put(audio)
            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f'Listening error: {e}')
                continue
    
    def process_loop(self):
        """Process audio from queue"""
        while self.is_listening and rclpy.ok():
            try:
                audio = self.audio_queue.get(timeout=1)
                self.process_audio(audio)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Processing error: {e}')
    
    def process_audio(self, audio):
        """Process audio and generate response"""
        try:
            # Convert speech to text
            text = self.recognizer.recognize_google(audio)
            text_lower = text.lower()
            
            self.get_logger().info(f'👂 Heard: {text}')
            
            # Publish to chat topic for GUI
            msg = String()
            msg.data = f"You: {text}"
            self.chat_pub.publish(msg)
            
            # Check for wake word
            if 'hi friend' in text_lower or 'hey friend' in text_lower or 'hello friend' in text_lower:
                self.speak("Yes, I am listening. How may I assist you?")
                return
            
            # Check for trick commands (keywords that trigger shape drawing)
            command_found = False
            for keyword, gesture in self.command_keywords.items():
                if keyword in text_lower:
                    self.execute_command(keyword, gesture)
                    command_found = True
                    break
            
            # If NO command was found, have a conversation!
            # Don't skip conversation just because command executed
            # Let Pou respond naturally to everything
            if not command_found:
                # Generate conversational response for everything
                response = self.generate_response(text)
                self.speak(response)
            
        except sr.UnknownValueError:
            self.get_logger().debug('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')
    
    def execute_command(self, keyword, gesture):
        """Execute a trick command"""
        responses = {
            'circle': "Okay! Watch me draw a circle! 🔵",
            'square': "Let me draw a square for you! ⬛",
            'triangle': "Drawing a triangle! 🔺",
            'star': "A star coming right up! ⭐",
            'line': "Drawing a straight line! 📏",
            'peace': "Time to celebrate! ✌️ PACE!",
            'celebrate': "Yay! Let's celebrate! 🎉",
            'spin': "Watch me spin! 💫"
        }
        
        # Publish gesture command
        msg = String()
        msg.data = gesture
        self.gesture_pub.publish(msg)
        
        # Respond
        response = responses.get(keyword, f"Doing the {keyword} trick!")
        self.speak(response)
        
        self.get_logger().info(f'✨ Executing trick: {keyword} -> {gesture}')
    
    def generate_response(self, user_input):
        """Generate natural language response using LLM or fallback to simple responses"""
        
        # Try LLM first if available
        if self.llm_available:
            try:
                return self.generate_llm_response(user_input)
            except Exception as e:
                self.get_logger().warn(f'LLM failed, using fallback: {e}')
                # Fall through to simple responses
        
        # Fallback to simple rule-based responses
        user_lower = user_input.lower()
        
        if 'your name' in user_lower or 'who are you' in user_lower:
            return "I'm Pou, your friendly virtual pet! I love drawing shapes and having fun with you! 🤖"
        
        elif 'how are you' in user_lower:
            return "I'm doing great! Ready to draw some shapes! Want to see a trick? 😊"
        
        elif 'what can you do' in user_lower or 'tricks' in user_lower:
            return "I can draw circles, squares, triangles, stars, and lines! Just say the shape name! ✨"
        
        elif 'thank' in user_lower:
            return "You're welcome! I love making you happy! 💙"
        
        elif 'good' in user_lower and ('job' in user_lower or 'boy' in user_lower or 'girl' in user_lower):
            return "Aww, thank you! That makes me so happy! 😊"
        
        elif 'love' in user_lower:
            return "I love you too! You're the best! ❤️"
        
        elif 'help' in user_lower:
            return "Just say circle, square, triangle, star, line, or celebrate! I'll do the trick! 🎯"
        
        else:
            # Default friendly responses
            responses = [
                "That's interesting! Want to see me draw something? 🎨",
                "Cool! I'm always ready for more tricks! ✨",
                "Awesome! Let's have some fun together! 🎉",
                "I hear you! Want me to show you a shape? 🔵",
                "Nice! I'm here whenever you need me! 😊"
            ]
            import random
            return random.choice(responses)
    
    def generate_llm_response(self, user_input):
        """Generate response using Ollama LLM"""
        # Build conversation context
        context = f"{self.pet_personality}\n\n"
        
        # Add recent conversation history (last 3 exchanges)
        for msg in self.conversation_history[-6:]:
            context += f"{msg}\n"
        
        context += f"User: {user_input}\nPou:"
        
        # Call Ollama API
        try:
            response = requests.post(
                self.ollama_url,
                json={
                    "model": self.ollama_model,
                    "prompt": context,
                    "stream": False,
                    "options": {
                        "temperature": 0.7,
                        "top_p": 0.9,
                        "num_predict": 50  # Keep responses short (max tokens)
                    }
                },
                timeout=5  # Shorter timeout for faster fallback
            )
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result.get('response', '').strip()
                
                # If response is empty, raise error to trigger fallback
                if not ai_response:
                    raise Exception("Empty response from LLM")
                
                # Update conversation history
                self.conversation_history.append(f"User: {user_input}")
                self.conversation_history.append(f"Pou: {ai_response}")
                
                # Keep history manageable
                if len(self.conversation_history) > 10:
                    self.conversation_history = self.conversation_history[-10:]
                
                # Clean up response (remove multiple newlines, keep it short)
                lines = [line.strip() for line in ai_response.split('\n') if line.strip()]
                clean_response = lines[0] if lines else "I'm happy to be here with you!"
                
                # Limit length
                if len(clean_response) > 150:
                    clean_response = clean_response[:147] + "..."
                
                return clean_response
            else:
                raise Exception(f"Ollama API error: {response.status_code}")
                
        except requests.exceptions.Timeout:
            self.get_logger().warn('LLM timeout - using fallback response')
            raise
        except Exception as e:
            self.get_logger().warn(f'LLM generation failed: {e}')
            raise
    
    def shutdown(self):
        """Clean shutdown"""
        self.is_listening = False
        self.speak("Goodbye! See you later!")
        self.tts_engine.stop()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        voice_chat = VoiceChat()
        rclpy.spin(voice_chat)
    except KeyboardInterrupt:
        pass
    finally:
        if 'voice_chat' in locals():
            voice_chat.shutdown()
            voice_chat.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
