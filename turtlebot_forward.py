#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveForward(Node):
    def __init__(self, target_distance=1.1):
        super().__init__('move_forward_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.target_distance = target_distance
        self.start_position = None
        self.movement_complete = False

        # Set forward speed
        self.motion_move = Twist()
        self.motion_move.linear.x = 0.2

        # Set stop command
        self.motion_stop = Twist()
        self.motion_stop.linear.x = 0.0

    def odom_callback(self, msg):
        current_position = msg.pose.pose.position.x
        if self.start_position is None:
            self.start_position = current_position

        traveled_distance = abs(current_position - self.start_position)
        
        if traveled_distance >= self.target_distance:
            self.pub.publish(self.motion_stop)
            self.get_logger().info("Target distance reached, stopping movement.")
            self.movement_complete = True
        else:
            self.pub.publish(self.motion_move)
            self.get_logger().info(f"Distance traveled: {traveled_distance:.2f} meters")

def main():
    rclpy.init()
    node = MoveForward(target_distance=3.2)
    
    while rclpy.ok() and not node.movement_complete:
        rclpy.spin_once(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

