#!/usr/bin/env python3
"""
Gesture Recognizer Node - Detects hand gestures using MediaPipe and OpenCV
Publishes recognized gestures to /pet/gesture topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import numpy as np


class GestureRecognizer(Node):
    def __init__(self):
        super().__init__('gesture_recognizer')
        
        # Publisher for recognized gestures
        self.gesture_pub = self.create_publisher(String, '/pet/gesture', 10)
        
        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # OpenCV camera setup
        self.cap = cv2.VideoCapture(2)  # Using camera index 2
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Timer for processing frames
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz
        
        # Last gesture to avoid spam
        self.last_gesture = ""
        self.gesture_cooldown = 0
        
        self.get_logger().info('Gesture Recognizer Node Started!')
        self.get_logger().info('Show hand gestures to the camera:')
        self.get_logger().info('  ✌️  Peace (2 fingers) - Print PACE!')
        self.get_logger().info('  ✊  Fist - Draw Square')
        self.get_logger().info('  ✋  Open Hand (5 fingers) - Draw Circle')
        self.get_logger().info('  👆  One Finger - Draw Line')
        self.get_logger().info('  🤘  Rock (thumb + pinky) - Draw Triangle')
        self.get_logger().info('  🤙  Shaka (thumb + pinky extended) - Draw Star')

    def count_extended_fingers(self, hand_landmarks):
        """Count how many fingers are extended"""
        # Finger tip and PIP landmarks
        finger_tips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]
        
        finger_pips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP
        ]
        
        extended_fingers = []
        
        # Check each finger (except thumb)
        for tip, pip in zip(finger_tips, finger_pips):
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[pip].y:
                extended_fingers.append(True)
            else:
                extended_fingers.append(False)
        
        # Check thumb (different logic)
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        
        # Thumb is extended if tip is farther from wrist than IP joint
        thumb_extended = (
            np.sqrt((thumb_tip.x - wrist.x)**2 + (thumb_tip.y - wrist.y)**2) >
            np.sqrt((thumb_ip.x - wrist.x)**2 + (thumb_ip.y - wrist.y)**2)
        )
        
        return extended_fingers, thumb_extended

    def recognize_gesture(self, hand_landmarks):
        """Recognize gesture based on finger positions"""
        extended_fingers, thumb_extended = self.count_extended_fingers(hand_landmarks)
        
        # Count total extended fingers
        num_fingers = sum(extended_fingers) + (1 if thumb_extended else 0)
        
        # Peace sign: index and middle finger extended
        if extended_fingers[0] and extended_fingers[1] and not extended_fingers[2] and not extended_fingers[3]:
            return "peace"
        
        # Fist: no fingers extended
        elif num_fingers == 0:
            return "fist"
        
        # Open hand: all fingers extended
        elif num_fingers >= 4:
            return "open_hand"
        
        # One finger: only index extended
        elif extended_fingers[0] and not extended_fingers[1] and not extended_fingers[2] and not extended_fingers[3]:
            return "one_finger"
        
        # Rock/Shaka: thumb and pinky extended
        elif thumb_extended and extended_fingers[3] and not extended_fingers[0] and not extended_fingers[1]:
            return "rock"
        
        # Three fingers (for star)
        elif num_fingers == 3:
            return "three_fingers"
        
        return "unknown"

    def process_frame(self):
        """Process camera frame and detect gestures"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        # Flip frame horizontally for natural interaction
        frame = cv2.flip(frame, 1)
        
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame with MediaPipe
        results = self.hands.process(rgb_frame)
        
        # Decrease cooldown
        if self.gesture_cooldown > 0:
            self.gesture_cooldown -= 1
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Recognize gesture
                gesture = self.recognize_gesture(hand_landmarks)
                
                # Publish if gesture changed and cooldown expired
                if gesture != "unknown" and gesture != self.last_gesture and self.gesture_cooldown == 0:
                    msg = String()
                    msg.data = gesture
                    self.gesture_pub.publish(msg)
                    self.get_logger().info(f'Gesture detected: {gesture}')
                    self.last_gesture = gesture
                    self.gesture_cooldown = 15  # 1.5 seconds cooldown at 10 Hz
                
                # Display gesture on frame
                cv2.putText(frame, f'Gesture: {gesture}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(frame, 'No hand detected', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Show frame
        cv2.imshow('Virtual Pet - Hand Gesture Recognition', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        """Cleanup resources"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
