#!/usr/bin/env python3
"""
Camera Test Script - Verify your webcam is working before running the full system
"""

import cv2
import sys

print("=" * 50)
print("Virtual Pet - Camera Test")
print("=" * 50)
print("\nTrying to open webcam...")

# Try to open camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ ERROR: Could not open webcam!")
    print("\nTroubleshooting:")
    print("1. Check if webcam is connected")
    print("2. Try: ls /dev/video*")
    print("3. Check webcam permissions")
    print("4. Try another camera index (change 0 to 1 or 2)")
    sys.exit(1)

print("✓ Webcam opened successfully!")
print("\nCamera Info:")
print(f"  Resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
print(f"  FPS: {int(cap.get(cv2.CAP_PROP_FPS))}")

print("\nShowing camera feed...")
print("Press 'q' to quit")
print("=" * 50)

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to capture frame")
        break
    
    # Flip frame for natural interaction
    frame = cv2.flip(frame, 1)
    
    # Add text
    cv2.putText(frame, "Camera Test - Press 'q' to quit", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, "If you can see this, camera is working!", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Display
    cv2.imshow('Virtual Pet - Camera Test', frame)
    
    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\n✓ Camera test complete!")
print("If you saw the video feed, your camera is ready for the virtual pet!")
