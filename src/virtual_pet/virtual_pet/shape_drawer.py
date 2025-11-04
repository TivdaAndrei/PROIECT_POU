#!/usr/bin/env python3
"""
Shape Drawer Node - Controls TurtleBot3 to draw shapes based on gestures
Subscribes to /pet/gesture and publishes velocity commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnEntity
import math
import time


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
        
        # Publisher for robot velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Service client for spawning text in Gazebo
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        
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
        
        self.get_logger().info('Shape Drawer Node Started!')
        self.get_logger().info('Ready to draw shapes with the virtual pet!')

    def gesture_callback(self, msg):
        """Handle incoming gesture messages"""
        gesture = msg.data
        self.get_logger().info(f'Received gesture: {gesture}')
        
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
            twist = Twist()
            twist.angular.z = 2.0  # Fast spin!
            self.cmd_vel_pub.publish(twist)
            self.shape_step += 1
        else:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()

    def spawn_pace_text(self):
        """Spawn PACE! text in Gazebo"""
        self.peace_count += 1
        
        # SDF model for text
        sdf_model = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='pace_text_{self.peace_count}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.5 0.1 0.001</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0.5 0 1</ambient>
          <diffuse>1 0.5 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
        request = SpawnEntity.Request()
        request.name = f'pace_text_{self.peace_count}'
        request.xml = sdf_model
        request.robot_namespace = ''
        request.initial_pose.position.x = float(self.peace_count * 0.5)
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.01
        
        # Call service asynchronously
        if self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            future = self.spawn_entity_client.call_async(request)
            self.get_logger().info('✌️ Spawning PACE! text in Gazebo')
        else:
            self.get_logger().warn('Gazebo spawn service not available')

    def draw_square(self):
        """Draw a square"""
        self.get_logger().info('Drawing SQUARE ⬛')
        self.is_drawing = True
        self.current_shape = "square"
        self.shape_step = 0
        self.timer = self.create_timer(0.05, self.square_step)

    def square_step(self):
        """Execute one step of square drawing"""
        side = self.shape_step // 100  # 4 sides
        progress = self.shape_step % 100
        
        if side >= 4:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Square complete!')
            return
        
        twist = Twist()
        if progress < 80:  # Move forward
            twist.linear.x = self.linear_speed
        elif progress < 100:  # Turn 90 degrees
            twist.angular.z = self.angular_speed * 2.0
        
        self.cmd_vel_pub.publish(twist)
        self.shape_step += 1

    def draw_circle(self):
        """Draw a circle"""
        self.get_logger().info('Drawing CIRCLE ⭕')
        self.is_drawing = True
        self.current_shape = "circle"
        self.shape_step = 0
        self.timer = self.create_timer(0.05, self.circle_step)

    def circle_step(self):
        """Execute one step of circle drawing"""
        if self.shape_step < 200:  # Complete circle
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
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
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_vel_pub.publish(twist)
            self.shape_step += 1
        else:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Line complete!')

    def draw_triangle(self):
        """Draw a triangle"""
        self.get_logger().info('Drawing TRIANGLE 🔺')
        self.is_drawing = True
        self.current_shape = "triangle"
        self.shape_step = 0
        self.timer = self.create_timer(0.05, self.triangle_step)

    def triangle_step(self):
        """Execute one step of triangle drawing"""
        side = self.shape_step // 100  # 3 sides
        progress = self.shape_step % 100
        
        if side >= 3:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Triangle complete!')
            return
        
        twist = Twist()
        if progress < 80:  # Move forward
            twist.linear.x = self.linear_speed
        elif progress < 100:  # Turn 120 degrees
            twist.angular.z = self.angular_speed * 2.5
        
        self.cmd_vel_pub.publish(twist)
        self.shape_step += 1

    def draw_star(self):
        """Draw a 5-pointed star"""
        self.get_logger().info('Drawing STAR ⭐')
        self.is_drawing = True
        self.current_shape = "star"
        self.shape_step = 0
        self.timer = self.create_timer(0.05, self.star_step)

    def star_step(self):
        """Execute one step of star drawing"""
        point = self.shape_step // 100  # 5 points
        progress = self.shape_step % 100
        
        if point >= 5:
            self.stop_robot()
            self.is_drawing = False
            self.timer.cancel()
            self.get_logger().info('Star complete!')
            return
        
        twist = Twist()
        if progress < 70:  # Move forward
            twist.linear.x = self.linear_speed
        elif progress < 100:  # Turn 144 degrees (exterior angle of star)
            twist.angular.z = self.angular_speed * 3.0
        
        self.cmd_vel_pub.publish(twist)
        self.shape_step += 1

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


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
