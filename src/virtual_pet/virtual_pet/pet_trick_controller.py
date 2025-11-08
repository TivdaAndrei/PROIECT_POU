#!/usr/bin/env python3
"""
Advanced Pet Trick Controller - Handles special pet tricks like follow, fetch, sit, dance
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped, Point
from nav_msgs.msg import Odometry
import math
import time


class PetTrickController(Node):
    def __init__(self):
        super().__init__('pet_trick_controller')
        
        # Subscriber for trick commands
        self.trick_sub = self.create_subscription(
            String,
            '/pet/trick',
            self.trick_callback,
            10
        )
        
        # Subscriber for finger pointing direction
        self.pointing_sub = self.create_subscription(
            Point,
            '/pet/pointing_direction',
            self.pointing_callback,
            10
        )
        
        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Publisher for status messages
        self.status_pub = self.create_publisher(String, '/pet/status', 10)
        
        # State variables
        self.current_trick = None
        self.trick_step = 0
        self.timer = None
        self.is_performing = False
        
        # Follow mode variables
        self.follow_mode = False
        self.target_direction = None
        self.current_position = None
        self.current_orientation = None
        
        # Movement parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        
        self.get_logger().info('🎭 Pet Trick Controller Started!')
        self.get_logger().info('Available tricks: follow, fetch, sit, dance, spin, wiggle')
    
    def odom_callback(self, msg):
        """Track robot position and orientation"""
        self.current_position = msg.pose.pose.position
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def pointing_callback(self, msg):
        """Receive finger pointing direction"""
        if self.follow_mode:
            # msg contains normalized direction vector
            self.target_direction = msg
            self.get_logger().debug(f'Following direction: ({msg.x:.2f}, {msg.y:.2f})')
    
    def trick_callback(self, msg):
        """Handle incoming trick commands"""
        trick = msg.data.lower()
        self.get_logger().info(f'🎭 Received trick command: {trick}')
        
        if self.is_performing and trick != "stop":
            self.get_logger().info('Already performing trick, ignoring...')
            return
        
        if trick == "follow":
            self.start_follow()
        elif trick == "stop_follow":
            self.stop_follow()
        elif trick == "sit":
            self.perform_sit()
        elif trick == "dance":
            self.perform_dance()
        elif trick == "spin":
            self.perform_spin()
        elif trick == "wiggle":
            self.perform_wiggle()
        elif trick == "fetch":
            self.perform_fetch()
        elif trick == "play_dead":
            self.perform_play_dead()
        elif trick == "stop":
            self.stop_current_trick()
    
    def start_follow(self):
        """Enter follow mode - Pou follows finger direction"""
        self.follow_mode = True
        self.is_performing = True
        self.get_logger().info('👉 Follow mode activated! Point your finger!')
        
        status = String()
        status.data = "Following your finger! 👆"
        self.status_pub.publish(status)
        
        # Create timer for follow updates
        self.timer = self.create_timer(0.1, self.follow_step)
    
    def stop_follow(self):
        """Exit follow mode"""
        self.follow_mode = False
        self.is_performing = False
        if self.timer:
            self.timer.cancel()
        self.stop_robot()
        self.get_logger().info('Follow mode deactivated')
    
    def follow_step(self):
        """Update Pou's movement to follow finger direction"""
        if not self.follow_mode or self.target_direction is None:
            return
        
        # Calculate angle to target direction
        target_angle = math.atan2(self.target_direction.y, self.target_direction.x)
        
        if self.current_orientation is not None:
            # Calculate angle difference
            angle_diff = target_angle - self.current_orientation
            
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Create velocity command
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = 'base_footprint'
            
            # If facing roughly the right direction, move forward
            if abs(angle_diff) < 0.3:  # ~17 degrees
                twist_stamped.twist.linear.x = self.linear_speed * 0.7
                twist_stamped.twist.angular.z = angle_diff * 0.5
            else:
                # Just turn
                twist_stamped.twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            
            self.cmd_vel_pub.publish(twist_stamped)
    
    def perform_sit(self):
        """Pou 'sits' - stops and waits"""
        self.is_performing = True
        self.get_logger().info('🪑 Pou is sitting!')
        
        status = String()
        status.data = "Sitting like a good pet! 🐾"
        self.status_pub.publish(status)
        
        self.stop_robot()
        
        # Stay sitting for 3 seconds
        self.timer = self.create_timer(3.0, self.end_sit)
    
    def end_sit(self):
        """End sitting trick"""
        self.is_performing = False
        if self.timer:
            self.timer.cancel()
        self.get_logger().info('Pou stood up!')
    
    def perform_dance(self):
        """Pou dances - alternating turns and forward movements"""
        self.is_performing = True
        self.current_trick = "dance"
        self.trick_step = 0
        self.get_logger().info('💃 Pou is dancing!')
        
        status = String()
        status.data = "Dancing! Watch me move! 🕺"
        self.status_pub.publish(status)
        
        self.timer = self.create_timer(0.1, self.dance_step)
    
    def dance_step(self):
        """Execute dance moves"""
        if self.trick_step >= 200:  # Dance for ~20 seconds
            self.stop_robot()
            self.is_performing = False
            self.timer.cancel()
            self.get_logger().info('Dance complete!')
            return
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        
        # Pattern: forward, spin left, forward, spin right
        phase = (self.trick_step // 25) % 4
        
        if phase == 0 or phase == 2:  # Forward
            twist_stamped.twist.linear.x = self.linear_speed * 0.5
        elif phase == 1:  # Spin left
            twist_stamped.twist.angular.z = self.angular_speed * 1.5
        else:  # Spin right
            twist_stamped.twist.angular.z = -self.angular_speed * 1.5
        
        self.cmd_vel_pub.publish(twist_stamped)
        self.trick_step += 1
    
    def perform_spin(self):
        """Pou spins around happily"""
        self.is_performing = True
        self.trick_step = 0
        self.get_logger().info('🌀 Pou is spinning!')
        
        status = String()
        status.data = "Wheee! Spinning around! 🌀"
        self.status_pub.publish(status)
        
        self.timer = self.create_timer(0.05, self.spin_step)
    
    def spin_step(self):
        """Execute spin"""
        if self.trick_step >= 80:  # Full 360 spin
            self.stop_robot()
            self.is_performing = False
            self.timer.cancel()
            return
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist.angular.z = self.angular_speed * 2.0
        
        self.cmd_vel_pub.publish(twist_stamped)
        self.trick_step += 1
    
    def perform_wiggle(self):
        """Pou wiggles back and forth"""
        self.is_performing = True
        self.trick_step = 0
        self.get_logger().info('🎵 Pou is wiggling!')
        
        status = String()
        status.data = "Wiggle wiggle! 🎵"
        self.status_pub.publish(status)
        
        self.timer = self.create_timer(0.1, self.wiggle_step)
    
    def wiggle_step(self):
        """Execute wiggle motion"""
        if self.trick_step >= 40:  # Wiggle for 4 seconds
            self.stop_robot()
            self.is_performing = False
            self.timer.cancel()
            return
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        
        # Alternate left and right
        if (self.trick_step // 5) % 2 == 0:
            twist_stamped.twist.angular.z = self.angular_speed * 0.8
        else:
            twist_stamped.twist.angular.z = -self.angular_speed * 0.8
        
        self.cmd_vel_pub.publish(twist_stamped)
        self.trick_step += 1
    
    def perform_fetch(self):
        """Pou runs forward to 'fetch' something"""
        self.is_performing = True
        self.trick_step = 0
        self.get_logger().info('🎾 Pou is fetching!')
        
        status = String()
        status.data = "I'll get it! 🎾"
        self.status_pub.publish(status)
        
        self.timer = self.create_timer(0.05, self.fetch_step)
    
    def fetch_step(self):
        """Execute fetch - run forward quickly"""
        if self.trick_step >= 100:  # Run for ~5 seconds
            self.stop_robot()
            self.is_performing = False
            self.timer.cancel()
            
            status = String()
            status.data = "Got it! 🎉"
            self.status_pub.publish(status)
            return
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist.linear.x = self.linear_speed * 1.5  # Faster!
        
        self.cmd_vel_pub.publish(twist_stamped)
        self.trick_step += 1
    
    def perform_play_dead(self):
        """Pou plays dead - completely still"""
        self.is_performing = True
        self.get_logger().info('💀 Pou is playing dead!')
        
        status = String()
        status.data = "Playing dead... 💀"
        self.status_pub.publish(status)
        
        self.stop_robot()
        self.timer = self.create_timer(5.0, self.wake_up)
    
    def wake_up(self):
        """Wake up from playing dead"""
        self.is_performing = False
        if self.timer:
            self.timer.cancel()
        
        status = String()
        status.data = "Just kidding! I'm alive! 😄"
        self.status_pub.publish(status)
        
        self.get_logger().info('Pou woke up!')
    
    def stop_current_trick(self):
        """Stop whatever trick is being performed"""
        self.is_performing = False
        self.follow_mode = False
        if self.timer:
            self.timer.cancel()
        self.stop_robot()
        self.get_logger().info('Stopped current trick')
    
    def stop_robot(self):
        """Send stop command"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    trick_controller = PetTrickController()
    
    try:
        rclpy.spin(trick_controller)
    except KeyboardInterrupt:
        pass
    finally:
        trick_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
