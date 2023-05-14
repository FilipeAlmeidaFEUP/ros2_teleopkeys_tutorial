#!/usr/bin/env python3
from pynput import keyboard
from pynput.keyboard import Key

import math
import random

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions

class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")

        # If false, robot performs random pathing mode
        # If true, robot performs keyboard control mode, but first make sure the keyboard reader node is running
        self.use_keyboard = False

        # Predefined speed for the robot
        self.linear_speed = 0.5
        self.angular_speed = 1.5

        # Minimum distance from obstacles in every direction
        # If lower performs an action depending on the mode
        self.min_distance = 0.2

        # For keyboard control mode, if true linear speed is halfed
        self.slow_down = False

        # For random pathing model, if >0 rotate robot only has anguçar speed
        self.rotation_iterations_left = 0

        # Save which keys are pressed
        self.keys_pressed = {
            'up':False,
            'down':False,
            'left':False,
            'right':False
        }

        # **** Create publishers ****
        self.pub:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)

        self.create_subscription(Collisions, "/collisions", self.processCollisions, 1)
        
        # Only in keyboard control mode
        if self.use_keyboard:
            self.create_subscription(String, "/teleopkeys", self.processKeystrokes, 10)
        # ******************************

    # Change the speed of the robot
    def change_speed(self, publisher, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        publisher.publish(twist_msg)

    # Send a request to move a model
    def move_model(self, model_name, x, y, theta):
        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = model_name
        request.pose = Pose2D()
        request.pose.x = x
        request.pose.y = y
        request.pose.theta = theta
        client.call_async(request)
    
    # Handle LiDAR data
    def processLiDAR(self, data):
        if self.use_keyboard:
            self.check_closest_obstacle(data)
        else:
            self.random_path(data)

    def random_path(self, data):
        if self.rotation_iterations_left == 0:

            # Find reading from laser in front
            front_laser = data.ranges[math.floor(len(data.ranges) / 2 + 0.5)]

            if front_laser < self.min_distance:

                # Override predefined angular speed to 1.570796327 or -1.570796327 rad/s
                self.angular_speed = 1.570796327 * ((random.randint(0, 1) * 2) - 1)

                # Lidar readings arrive every 0.1s
                # If rotate for 10 iterations, 
                # total rotation = 1.570796327 rad/s * (0.1s * 10) = 1.570796327 rad = 90º
                self.rotation_iterations_left = 10
            else:
                self.change_speed(self.pub, self.linear_speed, 0.0)
                return
        self.rotation_iterations_left -= 1
        self.change_speed(self.pub, 0.0, self.angular_speed)

    def check_closest_obstacle(self,data):
        # Find the closest reading
        min_laser = min(data.ranges)
        if min_laser < self.min_distance:

            # Force the robot to slow down
            self.slow_down = True

            # Calculate the the direction of the closest laser to print a warning
            # by dividing the lasers in front, left, right back sections
            n_lasers = len(data.ranges)
            quadrant_range = n_lasers // 8
            min_index = data.ranges.index(min_laser)
            if abs(min_index - (n_lasers // 2)) <= quadrant_range: 
                self.get_logger().info('Warning! Wall in the front!')
            if abs(min_index - (n_lasers // 4)) <= quadrant_range:
                self.get_logger().info('Warning! Wall to the right!')
            if abs(min_index - (n_lasers - (n_lasers // 4))) <= quadrant_range:
                self.get_logger().info('Warning! Wall to the left!')
            if min_index <= quadrant_range or min_index >= n_lasers - quadrant_range: 
                self.get_logger().info('Warning! Wall in the back!')
        else: self.slow_down = False
    
    # Process collisions
    def processCollisions(self, data):
        if len(data.collisions) > 0:
            self.rotation_iterations_left = 0
            
            self.change_speed(self.pub, 0.0, 0.0)

            # Move robot to initial position
            self.move_model("serp", 0.0, 0.0, -1.57079632679)

    #Process keystroke messages
    def processKeystrokes(self, data):
        str = data.data
        self.register_keystroke(str[2:], str[0] == 'p')
        
        self.teleopkeys()
    
    # Use the keystrokes received to save which keys are pressed
    def register_keystroke(self, key, is_pressed):
        match key:
            case 'up': self.keys_pressed['up'] = is_pressed
            case 'down': self.keys_pressed['down'] = is_pressed
            case 'left': self.keys_pressed['left'] = is_pressed
            case 'right': self.keys_pressed['right'] = is_pressed

    # Based on which keys are pressed, change the robots speed
    def teleopkeys(self):
        linear_speed = 0.0
        angular_speed = 0.0
        if self.keys_pressed['up']: linear_speed += self.linear_speed
        if self.keys_pressed['down']: linear_speed -= self.linear_speed
        if self.keys_pressed['left']: angular_speed += self.angular_speed
        if self.keys_pressed['right']: angular_speed -= self.angular_speed

        if self.slow_down: linear_speed /= 2

        self.change_speed(self.pub, linear_speed, angular_speed)

def main(args = None):
    rclpy.init()
    
    serp = SerpController()

    rclpy.spin(serp)

if __name__ == "__main__":
    main()
