#!/usr/bin/env python3
"""
Shape Drawer Node - Controls TurtleBot3 to draw shapes based on gestures
Subscribes to /pet/gesture and publishes velocity commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
import random
import subprocess


class ShapeDrawer(Node):
    def __init__(self):
        super().__init__('shape_drawer')
        
        # Subscriber for gestures
        self.gesture_sub = self.create_subscription(
            String,
            '/pet/gesture',
            self.gesture_callback,
            10
        )
        
        # Publisher for robot velocity (TwistStamped for Gazebo bridge)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Publisher for trail markers (visualization)
        self.marker_pub = self.create_publisher(Marker, '/pet/trail_markers', 10)
        
        # Subscriber for robot position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Check if Gazebo is available
        self.gazebo_available = self.check_gazebo_available()
        
        # State variables
        self.is_drawing = False
        self.current_shape = None
        self.shape_step = 0
        self.timer = None
        
        # Drawing parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.side_length = 1.0  # meters for shapes
        
        self.peace_count = 0  # Counter for PACE! text instances
        
        # Trail tracking
        self.last_position = None
        self.current_position = None
        self.trail_counter = 0
        self.trail_distance_threshold = 0.02  # Spawn trail marker every 2cm (more dense trail)
        
        # Color mapping for gestures
        self.gesture_colors = {
            'peace': (1.0, 0.0, 1.0, 1.0),      # Magenta
            'fist': (1.0, 0.0, 0.0, 1.0),       # Red
            'open_hand': (0.0, 1.0, 0.0, 1.0),  # Green
            'one_finger': (0.0, 0.0, 1.0, 1.0), # Blue
            'rock': (1.0, 0.5, 0.0, 1.0),       # Orange
            'three_fingers': (1.0, 1.0, 0.0, 1.0), # Yellow
        }
        self.current_color = (1.0, 1.0, 1.0, 1.0)  # Default white
        
        self.get_logger().info('Shape Drawer Node Started!')
        self.get_logger().info('Ready to draw shapes with the virtual pet!')
    
    def check_gazebo_available(self):
        """Check if Gazebo is running"""
        try:
            result = subprocess.run(
                ['gz', 'service', '-l'],
                capture_output=True,
                text=True,
                timeout=2.0
            )
            if '/world/default/create' in result.stdout:
                self.get_logger().info('✅ Gazebo simulation detected!')
                return True
            else:
                self.get_logger().warn('⚠️ Gazebo not detected - trails will not appear')
                return False
        except Exception as e:
            self.get_logger().warn(f'⚠️ Could not detect Gazebo: {e}')
            return False

    def odom_callback(self, msg):
        """Track robot position for trail drawing"""
        self.current_position = msg.pose.pose.position
        
        # If drawing, spawn trail markers
        if self.is_drawing and self.last_position is not None:
            distance = math.sqrt(
                (self.current_position.x - self.last_position.x)**2 +
                (self.current_position.y - self.last_position.y)**2
            )
            
            if distance >= self.trail_distance_threshold:
                self.spawn_trail_marker()
                self.last_position = self.current_position
        elif self.is_drawing and self.last_position is None:
            self.last_position = self.current_position

    def gesture_callback(self, msg):
        """Handle incoming gesture messages"""
        gesture = msg.data
        self.get_logger().info(f'Received gesture: {gesture}')
        
        # Update color based on gesture
        if gesture in self.gesture_colors:
            self.current_color = self.gesture_colors[gesture]
        
        # Don't start new shape if already drawing
        if self.is_drawing:
            self.get_logger().info('Already drawing, ignoring gesture')
            return
        
        # Map gestures to actions
        if gesture == "peace":
            self.spawn_pace_text()
            self.celebrate()  # Do a little spin!
        elif gesture == "fist":
            self.draw_square()
        elif gesture == "open_hand":
            self.draw_circle()
        elif gesture == "one_finger":
            self.draw_line()
        elif gesture == "rock":
            self.draw_triangle()
        elif gesture == "three_fingers":
            self.draw_star()

    def celebrate(self):
        """Make the pet do a happy spin when peace sign is shown"""
        self.get_logger().info('🎉 PACE! Celebrating with a spin! 🎉')
        self.is_drawing = True
        self.current_shape = "celebrate"
        self.shape_step = 0
        
        # Create timer for celebration animation
        self.timer = self.create_timer(0.05, self.celebrate_step)

    def celebrate_step(self):
        """Execute celebration spin"""
        if self.shape_step < 40:  # Spin for 2 seconds
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = 'base_footprint'
            twist_stamped.twist.angular.z = 2.0  # Fast spin!
            self.cmd_vel_pub.publish(twist_stamped)
            self.shape_step += 1
        else:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()

    def spawn_pace_text(self):
        """Spawn PACE! text in Gazebo using gz command"""
        self.peace_count += 1
        
        if not self.gazebo_available:
            self.get_logger().info('✌️ PACE! (Gazebo not available for visual)')
            return
        
        # Simple visual marker for PACE
        x_pos = float(self.peace_count * 0.5)
        
        # SDF model for a bright marker
        sdf_model = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='pace_marker_{self.peace_count}'>
    <static>true</static>
    <pose>{x_pos} 0 0.2 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.3 0.3 0.4</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 1 1</ambient>
          <diffuse>1 0 1 1</diffuse>
          <emissive>0.8 0 0.8 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
        # Spawn using gz command
        try:
            subprocess.Popen(
                ['gz', 'service', '-s', '/world/default/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000',
                 '--req', f'sdf: "{sdf_model.replace(chr(10), " ")}"'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info('✌️ PACE! Spawned celebration marker!')
        except Exception as e:
            self.get_logger().warn(f'Could not spawn PACE marker: {e}')

    def spawn_trail_marker(self):
        """Spawn a colorful trail marker for RViz visualization"""
        if self.current_position is None:
            return
        
        self.trail_counter += 1
        r, g, b, a = self.current_color
        
        x = float(self.current_position.x)
        y = float(self.current_position.y)
        
        # Create RViz Marker (this ALWAYS works)
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trail"
        marker.id = self.trail_counter
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.075  # Just above ground
        marker.pose.orientation.w = 1.0
        
        # Scale (size of sphere)
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        # Color
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        
        # Duration (permanent)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Publish to RViz
        self.marker_pub.publish(marker)
        
        if self.trail_counter % 20 == 0:
            self.get_logger().info(f'🎨 {self.trail_counter} trail markers spawned')
        
        # Try Gazebo spawning as well (optional, may fail)
        if self.gazebo_available:
            try:
                sdf_model = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='trail_{self.trail_counter}'>
    <static>true</static>
    <pose>{x} {y} 0.075 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>0.075</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>1 1 1 1</specular>
          <emissive>{r*0.8} {g*0.8} {b*0.8} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
                
                sdf_inline = sdf_model.replace('\n', ' ').replace('"', '\\"')
                subprocess.Popen(
                    ['gz', 'service', '-s', '/world/default/create',
                     '--reqtype', 'gz.msgs.EntityFactory',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '100',
                     '--req', f'sdf: "{sdf_inline}"'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except Exception:
                pass  # Silently fail for Gazebo


    def draw_square(self):
        """Draw a square"""
        self.get_logger().info('Drawing SQUARE ⬛')
        self.is_drawing = True
        self.current_shape = "square"
        self.shape_step = 0
        self.side_time = self.side_length / self.linear_speed  # Time to travel one side
        self.turn_time = (math.pi / 2) / self.angular_speed  # Time to turn 90 degrees
        self.timer = self.create_timer(0.1, self.square_step)

    def square_step(self):
        """Execute one step of square drawing"""
        # Each side: move + turn
        steps_per_side = int((self.side_time + self.turn_time) / 0.1)
        turn_start = int(self.side_time / 0.1)
        
        side = self.shape_step // steps_per_side
        progress = self.shape_step % steps_per_side
        
        if side >= 4:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Square complete!')
            return
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        
        if progress < turn_start:  # Moving forward
            twist_stamped.twist.linear.x = self.linear_speed
        else:  # Turning
            twist_stamped.twist.angular.z = self.angular_speed
        
        self.cmd_vel_pub.publish(twist_stamped)
        self.shape_step += 1

    def draw_circle(self):
        """Draw a circle"""
        self.get_logger().info('Drawing CIRCLE ⭕')
        self.is_drawing = True
        self.current_shape = "circle"
        self.shape_step = 0
        # For a circle: v = r*w, so angular velocity = linear / radius
        # We want radius ~0.5m
        self.circle_radius = 0.5
        self.circle_angular = self.linear_speed / self.circle_radius
        self.circle_time = (2 * math.pi) / self.circle_angular  # Time for full circle
        self.timer = self.create_timer(0.1, self.circle_step)

    def circle_step(self):
        """Execute one step of circle drawing"""
        max_steps = int(self.circle_time / 0.1)
        
        if self.shape_step < max_steps:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = 'base_footprint'
            twist_stamped.twist.linear.x = self.linear_speed
            twist_stamped.twist.angular.z = self.circle_angular
            self.cmd_vel_pub.publish(twist_stamped)
            self.shape_step += 1
        else:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Circle complete!')

    def draw_line(self):
        """Draw a straight line"""
        self.get_logger().info('Drawing LINE ➡️')
        self.is_drawing = True
        self.current_shape = "line"
        self.shape_step = 0
        self.timer = self.create_timer(0.05, self.line_step)

    def line_step(self):
        """Execute one step of line drawing"""
        if self.shape_step < 100:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = 'base_footprint'
            twist_stamped.twist.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(twist_stamped)
            self.shape_step += 1
        else:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Line complete!')

    def draw_triangle(self):
        """Draw an equilateral triangle"""
        self.get_logger().info('Drawing TRIANGLE 🔺')
        self.is_drawing = True
        self.current_shape = "triangle"
        self.shape_step = 0
        self.side_time = self.side_length / self.linear_speed
        # For equilateral triangle, each exterior angle is 120 degrees = 2.094 radians
        self.turn_time = (2 * math.pi / 3) / self.angular_speed
        self.timer = self.create_timer(0.1, self.triangle_step)

    def triangle_step(self):
        """Execute one step of triangle drawing"""
        steps_per_side = int((self.side_time + self.turn_time) / 0.1)
        turn_start = int(self.side_time / 0.1)
        
        side = self.shape_step // steps_per_side
        progress = self.shape_step % steps_per_side
        
        if side >= 3:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Triangle complete!')
            return
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        
        if progress < turn_start:  # Moving forward
            twist_stamped.twist.linear.x = self.linear_speed
        else:  # Turning 120 degrees
            twist_stamped.twist.angular.z = self.angular_speed
        
        self.cmd_vel_pub.publish(twist_stamped)
        self.shape_step += 1

    def draw_star(self):
        """Draw a 5-pointed star"""
        self.get_logger().info('Drawing STAR ⭐')
        self.is_drawing = True
        self.current_shape = "star"
        self.shape_step = 0
        self.side_time = self.side_length / self.linear_speed
        # For a 5-pointed star, turn 144 degrees (720/5) = 2.513 radians
        self.turn_time = (2 * math.pi * 2 / 5) / self.angular_speed
        self.timer = self.create_timer(0.1, self.star_step)

    def star_step(self):
        """Execute one step of star drawing"""
        steps_per_point = int((self.side_time + self.turn_time) / 0.1)
        turn_start = int(self.side_time / 0.1)
        
        point = self.shape_step // steps_per_point
        progress = self.shape_step % steps_per_point
        
        if point >= 5:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Star complete!')
            return
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        
        if progress < turn_start:  # Moving forward
            twist_stamped.twist.linear.x = self.linear_speed
        else:  # Turning 144 degrees
            twist_stamped.twist.angular.z = self.angular_speed
        
        self.cmd_vel_pub.publish(twist_stamped)
        self.shape_step += 1
        self.shape_step += 1

    def stop_robot(self):
        """Stop the robot"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_footprint'
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDrawer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
