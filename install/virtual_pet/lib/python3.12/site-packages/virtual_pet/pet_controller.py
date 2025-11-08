#!/usr/bin/env python3
"""
Pet Controller Node - Main coordinator for the virtual pet
Monitors system health and provides status updates
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PetController(Node):
    def __init__(self):
        super().__init__('pet_controller')
        
        # Subscribe to gestures for monitoring
        self.gesture_sub = self.create_subscription(
            String,
            '/pet/gesture',
            self.gesture_monitor,
            10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/pet/status', 10)
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.gesture_count = 0
        
        self.get_logger().info('🐾 Virtual Pet Controller Started! 🐾')
        self.get_logger().info('Your virtual pet is ready to play!')
        self.get_logger().info('Show hand gestures to make it draw shapes!')

    def gesture_monitor(self, msg):
        """Monitor received gestures"""
        self.gesture_count += 1
        self.get_logger().info(f'Total gestures recognized: {self.gesture_count}')

    def publish_status(self):
        """Publish pet status"""
        msg = String()
        msg.data = f'Virtual Pet Active - {self.gesture_count} gestures processed'
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PetController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
