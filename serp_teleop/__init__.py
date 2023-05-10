#!/usr/bin/env python3

import pygame
from pygame.locals import *

from threading import Thread

from pynput import keyboard
from pynput.keyboard import Key

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions

class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")

        self.linear_speed = 0.5
        self.angular_speed = 1.5

        self.warning_range = 0.2
        self.slow_down = False

        self.keys_pressed = {
            'up':False,
            'down':False,
            'left':False,
            'right':False
        }


        self.vel = Twist()

        self.pub:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)

        self.create_subscription(Collisions, "/collisions", self.processCollisions, 1)

    def processLiDAR(self, data):

        min_laser = min(data.ranges)
        if min_laser < self.warning_range:
            self.slow_down = True
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
        
        linear_speed = 0.0
        angular_speed = 0.0
        if self.keys_pressed['up']: linear_speed += self.linear_speed
        if self.keys_pressed['down']: linear_speed -= self.linear_speed
        if self.keys_pressed['left']: angular_speed += self.angular_speed
        if self.keys_pressed['right']: angular_speed -= self.angular_speed

        if self.slow_down: linear_speed /= 2
        
        self.vel.linear.x = linear_speed
        self.vel.angular.z = angular_speed

        self.pub.publish(self.vel)

    def processCollisions(self, data):
        if len(data.collisions) > 0:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.pub.publish(vel)
            client = self.create_client(MoveModel, "/move_model")
            client.wait_for_service()
            request = MoveModel.Request()
            request.name = "serp"
            request.pose = Pose2D()
            request.pose.x = 0.0
            request.pose.y = 0.0
            request.pose.theta = -1.57079632679
            client.call_async(request)

    def spin(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while rclpy.ok() and listener.running:
                rclpy.spin_once(self, timeout_sec=0.1)

    def on_release(self, key):
        if key == Key.up: self.keys_pressed['up'] = False
        elif key == Key.down: self.keys_pressed['down'] = False
        elif key == Key.left: self.keys_pressed['left'] = False
        elif key == Key.right: self.keys_pressed['right'] = False

    def on_press(self, key):
        if key == Key.up: self.keys_pressed['up'] = True
        elif key == Key.down: self.keys_pressed['down'] = True
        elif key == Key.left: self.keys_pressed['left'] = True
        elif key == Key.right: self.keys_pressed['right'] = True

def main(args = None):
    rclpy.init()
    
    serp = SerpController()

    thread = Thread(target = serp.spin)
    thread.start()
    thread.join()

    rclpy.spin(serp)

if __name__ == "__main__":
    main()
