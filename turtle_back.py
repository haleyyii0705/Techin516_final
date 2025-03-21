#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self, speed=-0.2, distance=3.2):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed  # Set the speed (negative for moving backward)
        self.target_distance = distance  # Set the target distance to move
        self.movement_complete = False

    def move(self):
        print("Starting to move backward...")

        twist = Twist()
        twist.linear.x = self.speed  # Set the backward speed

        # Calculate how long to move based on speed and distance
        duration = self.target_distance / abs(self.speed)
        start_time = time.time()

        # Publish movement command
        while (time.time() - start_time) < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # Ensure the robot stops
        twist.linear.x = 0.0
        for _ in range(5):  # Publish stop command multiple times
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        print("Movement complete. TurtleBot stopped.")
        self.movement_complete = True

def main():
    rclpy.init()

    # Create a node instance and set the target distance
    node = TurtleBotController(speed=-0.1, distance=3.2)
    
    # Execute the move command
    node.move()

    # Give ROS time to process the stop command
    time.sleep(1)

    # Clean up and shut down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

