#!/usr/bin/env python3
"""
GUI Controller Node - Provides graphical interface for virtual pet control
Shows real-time alerts and provides manual control buttons
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, scrolledtext
from datetime import datetime
import threading


class PetGUI(Node):
    def __init__(self):
        super().__init__('pet_gui')
        
        # Publisher for manual gesture commands
        self.gesture_pub = self.create_publisher(String, '/pet/gesture', 10)
        
        # Subscriber for status messages
        self.status_sub = self.create_subscription(
            String,
            '/pet/status',
            self.status_callback,
            10
        )
        
        # Subscriber for gesture detection (for alerts)
        self.gesture_sub = self.create_subscription(
            String,
            '/pet/gesture',
            self.gesture_alert_callback,
            10
        )
        
        self.get_logger().info('🖥️ GUI Controller Node Started!')
        
        # Create GUI in main thread
        self.create_gui()
    
    def create_gui(self):
        """Create the GUI window"""
        self.window = tk.Tk()
        self.window.title("🤖 Virtual Pet Controller")
        self.window.geometry("800x600")
        self.window.configure(bg='#2b2b2b')
        
        # Title
        title_frame = tk.Frame(self.window, bg='#2b2b2b')
        title_frame.pack(pady=10)
        
        title_label = tk.Label(
            title_frame,
            text="🤖 Virtual Pet Control Center",
            font=('Arial', 24, 'bold'),
            bg='#2b2b2b',
            fg='#00ff00'
        )
        title_label.pack()
        
        # Status label
        self.status_label = tk.Label(
            title_frame,
            text="Status: Ready",
            font=('Arial', 12),
            bg='#2b2b2b',
            fg='#ffffff'
        )
        self.status_label.pack(pady=5)
        
        # Control buttons frame
        button_frame = tk.Frame(self.window, bg='#2b2b2b')
        button_frame.pack(pady=20)
        
        # Create gesture control buttons
        self.create_button_grid(button_frame)
        
        # Alert log frame
        log_frame = tk.Frame(self.window, bg='#2b2b2b')
        log_frame.pack(pady=10, padx=20, fill='both', expand=True)
        
        log_title = tk.Label(
            log_frame,
            text="📋 Real-Time Alerts",
            font=('Arial', 14, 'bold'),
            bg='#2b2b2b',
            fg='#ffffff'
        )
        log_title.pack()
        
        # Scrolled text for alerts
        self.alert_text = scrolledtext.ScrolledText(
            log_frame,
            height=15,
            width=90,
            bg='#1e1e1e',
            fg='#00ff00',
            font=('Courier', 10),
            wrap=tk.WORD
        )
        self.alert_text.pack(pady=5)
        
        # Clear button
        clear_btn = tk.Button(
            log_frame,
            text="🗑️ Clear Alerts",
            command=self.clear_alerts,
            bg='#ff5555',
            fg='white',
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=5
        )
        clear_btn.pack(pady=5)
        
        # Initial welcome message
        self.add_alert("System initialized. Ready to control virtual pet!", "INFO")
        self.add_alert("Use buttons below or hand gestures to control the robot.", "INFO")
    
    def create_button_grid(self, parent):
        """Create grid of control buttons"""
        buttons = [
            ("✌️ Peace\n(Celebrate)", "peace", "#ff00ff"),
            ("✊ Fist\n(Square)", "fist", "#ff0000"),
            ("🖐️ Open Hand\n(Circle)", "open_hand", "#00ff00"),
            ("☝️ One Finger\n(Line)", "one_finger", "#0000ff"),
            ("🤘 Rock\n(Triangle)", "rock", "#ff8800"),
            ("🤟 Three Fingers\n(Star)", "three_fingers", "#ffff00")
        ]
        
        row = 0
        col = 0
        for text, gesture, color in buttons:
            btn = tk.Button(
                parent,
                text=text,
                command=lambda g=gesture: self.send_gesture(g),
                bg=color,
                fg='white',
                font=('Arial', 12, 'bold'),
                width=15,
                height=3,
                relief='raised',
                bd=3
            )
            btn.grid(row=row, column=col, padx=10, pady=10)
            
            col += 1
            if col > 2:
                col = 0
                row += 1
    
    def send_gesture(self, gesture):
        """Send gesture command manually"""
        msg = String()
        msg.data = gesture
        self.gesture_pub.publish(msg)
        self.add_alert(f"Manual command sent: {gesture.upper()}", "COMMAND")
        self.get_logger().info(f'📤 Manual gesture sent: {gesture}')
    
    def status_callback(self, msg):
        """Update status from pet controller"""
        self.status_label.config(text=f"Status: {msg.data}")
    
    def gesture_alert_callback(self, msg):
        """Show alert when gesture is detected"""
        gesture = msg.data
        gesture_names = {
            'peace': '✌️ PEACE - Celebrating!',
            'fist': '✊ FIST - Drawing Square',
            'open_hand': '🖐️ OPEN HAND - Drawing Circle',
            'one_finger': '☝️ ONE FINGER - Drawing Line',
            'rock': '🤘 ROCK - Drawing Triangle',
            'three_fingers': '🤟 THREE FINGERS - Drawing Star'
        }
        
        alert_text = gesture_names.get(gesture, f'Unknown gesture: {gesture}')
        self.add_alert(f"Gesture detected: {alert_text}", "GESTURE")
    
    def add_alert(self, message, alert_type="INFO"):
        """Add alert to the log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Color coding
        colors = {
            "INFO": "#00ff00",
            "GESTURE": "#00ffff",
            "COMMAND": "#ffff00",
            "ERROR": "#ff5555"
        }
        color = colors.get(alert_type, "#ffffff")
        
        formatted_msg = f"[{timestamp}] [{alert_type}] {message}\n"
        
        self.alert_text.insert(tk.END, formatted_msg)
        self.alert_text.see(tk.END)  # Auto-scroll to bottom
        
        # Change text color for last line
        last_line_start = self.alert_text.index("end-2c linestart")
        self.alert_text.tag_add(alert_type, last_line_start, "end-1c")
        self.alert_text.tag_config(alert_type, foreground=color)
    
    def clear_alerts(self):
        """Clear the alert log"""
        self.alert_text.delete(1.0, tk.END)
        self.add_alert("Alert log cleared.", "INFO")
    
    def run(self):
        """Run the GUI main loop"""
        self.window.mainloop()


def main(args=None):
    rclpy.init(args=args)
    
    gui = PetGUI()
    
    # Run ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    spin_thread.start()
    
    # Run GUI in main thread
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
