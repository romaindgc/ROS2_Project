#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class node_cmdTurtle1(Node):

    def __init__(self):
        super().__init__('Follower')

        # Variables for the position
        self.pose_turtle1 = [0, 0, 0, 0, 0]
        self.pose_turtleFollower = [0, 0, 0, 0, 0]

        # Variables for the PID
        self.error_distance = 0
        self.error_distance_previous = 0
        self.error_angle = 0
        self.error_angle_previous = 0

        self.integrated_distance = 0
        self.integrated_angle = 0

        # PID Gains
        self.Kp_distance = 1
        self.Ki_distance = 0.01
        self.Kd_distance = 0.2

        self.Kp_angle = 3.0
        self.Ki_angle = 0.01
        self.Kd_angle = 0.2

        # Other parameters
        self.FD = 1.0  # Target distance
        self.timer_period = 0.1  # seconds

        # Publishers and Subscriptions
        self.pose_turtle1_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback_driver, 10)
        self.pose_turtle_follower_sub = self.create_subscription(Pose, "/turtleFollower/pose", self.pose_callback_follower, 10)
        self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/turtleFollower/cmd_vel", 10)

        self.timer_follow = self.create_timer(self.timer_period, self.followCommand)
        self.timer_print = self.create_timer(self.timer_period, self.print_pose_callback)
        self.get_logger().info("Shadowing started")
    
    def pose_callback_driver(self, pose: Pose):
        self.pose_turtle1 = [pose.x, pose.y, pose.theta, pose.linear_velocity, pose.angular_velocity]
      
    def pose_callback_follower(self, pose: Pose):
        self.pose_turtleFollower = [pose.x, pose.y, pose.theta]

    def print_pose_callback(self):
        self.get_logger().info(f"Position turtle1: x={self.pose_turtle1[0]}, y={self.pose_turtle1[1]}")
        self.get_logger().info(f"Position turtleFollower: x={self.pose_turtleFollower[0]}, y={self.pose_turtleFollower[1]}")

    def send_velocity_command(self, x, y, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(z)
        self.cmd_vel_turtle_follower_pub_.publish(msg)

    def followCommand(self):
        # Calculate errors
        self.error_distance = norme_euclidienne(self.pose_turtle1, self.pose_turtleFollower)
        self.error_angle = np.arctan2(self.pose_turtle1[1] - self.pose_turtleFollower[1], self.pose_turtle1[0] - self.pose_turtleFollower[0]) - self.pose_turtleFollower[2]
        
        # Normalize angle in [-pi,pi] for ensuring that the turtle turns in the shortest sens
        self.error_angle = np.arctan2(np.sin(self.error_angle), np.cos(self.error_angle))

        # PID for distance
        self.integrated_distance += self.error_distance * self.timer_period
        self.derivated_distance = (self.error_distance - self.error_distance_previous) / self.timer_period
        linear_speed = self.Kp_distance * self.error_distance + self.Ki_distance * self.integrated_distance + self.Kd_distance * self.derivated_distance

        # PID for angle
        self.integrated_angle += self.error_angle * self.timer_period
        self.derivated_angle = (self.error_angle - self.error_angle_previous) / self.timer_period
        angle_speed = self.Kp_angle * self.error_angle + self.Ki_angle * self.integrated_angle + self.Kd_angle * self.derivated_angle

        # Update previous errors
        self.error_distance_previous = self.error_distance
        self.error_angle_previous = self.error_angle

        # Tolerance zone
        tolerance = 0.1 #For avoiding perpetual oscillations, we add a tolerance to our aimed distance
        if self.error_distance > self.FD + tolerance:
            self.send_velocity_command(linear_speed, 0, angle_speed)
        elif self.error_distance < self.FD - tolerance:
            self.send_velocity_command(-linear_speed, 0, -angle_speed)
        else:
            self.send_velocity_command(0, 0, 0)

def norme_euclidienne(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def main(args=None):
    rclpy.init(args=args)
    node = node_cmdTurtle1()
    rclpy.spin(node)
    rclpy.shutdown()
