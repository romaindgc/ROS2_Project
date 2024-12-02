#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from .utils import *

class node_robotFollower(Node):

    def __init__(self):
        super().__init__('Follower')

        # Variables for the position
        self.pose_leader = Pos()
        self.pose_follower = Pos()

        #Setup offset follower
        self.pose_follower.set_offset(Pos(-4))

        # Other parameters
        self.FD = 2.5  # Target distance
        self.timer_period = 0.1  # seconds
        self.xOffset_follower = -4

        # Publishers and Subscriptions
        self.pose_leader_sub = self.create_subscription(Odometry, "/model/vehicle_blue/odometry", self.pose_leader.update_gz, 10)
        self.pose_follower_sub = self.create_subscription(Odometry, "/model/vehicle_green/odometry", self.pose_follower.update_gz, 10)
        
        self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/model/vehicle_green/cmd_vel", 10)

        self.timer_follow = self.create_timer(self.timer_period, self.send_velocity_command)
        self.get_logger().info("Shadowing started")

        #Defintion of the PID
        self.follower_cmd = PID_cmd(self.timer_period, dist_target=1.5,dist_gain=[1.0,0.01,0.2], angle_gain=[1.0,0.01,0.2])  

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.follower_cmd.commande(self.pose_follower, self.pose_leader)
        self.cmd_vel_turtle_follower_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = node_robotFollower()
    rclpy.spin(node)
    rclpy.shutdown()
