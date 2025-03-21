#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self, speed=0.2):
        super().__init__('turtlebot_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = speed  # Speed of the robot (m/s)

    # General movement function that can handle both forward and turning actions
    def move(self, linear_x=0.0, angular_z=0.0, duration=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        start_time = time.time()

        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # Stop movement once the duration has passed
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        time.sleep(1.0)  # Giving time for the last command to execute

    # Move the robot a specified distance forward
    def move_forward(self, distance):
        print(f"Moving forward {distance} meters...")
        duration = distance / self.speed
        self.move(linear_x=self.speed, duration=duration)
        print(f"Finished moving forward {distance} meters.\n")

    # Turn the robot left or right by a given angle (in degrees)
    def turn(self, direction='left', angle=90):
        print(f"Turning {direction} {angle} degrees...")
        
        angular_speed = 0.78  # radians per second
        if direction == 'left':
            angular_z = angular_speed
        elif direction == 'right':
            angular_z = -angular_speed
        else:
            print("Invalid direction! Use 'left' or 'right'.")
            return
        
        duration = (angle / 90) * 2.0  # Assuming it takes 2 seconds to turn 90 degrees
        self.move(angular_z=angular_z, duration=duration)
        print(f"Finished turning {direction}.\n")

def main():
    rclpy.init()
    bot = TurtleBotController(speed=0.2)

    # Path sequence using the new modular functions
    bot.move_forward(0.45)
    bot.turn('left', 90)
    bot.move_forward(1.65)
    bot.turn('left', 90)
    bot.move_forward(0.8)
    bot.turn('right', 90)
    bot.move_forward(1.1)
    bot.turn('right', 90)
    bot.move_forward(0.5)
    bot.turn('left', 90)
    bot.move_forward(0.45)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

