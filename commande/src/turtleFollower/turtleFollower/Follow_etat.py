#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np

class node_cmdEtat(Node):

    def __init__(self):
        super().__init__('Follower')

        # Variables pour la position
        self.pose_turtle1 = [0, 0, 0, 0, 0]  # x, y, theta, linear_velocity, angular_velocity
        self.pose_turtleFollower = [0, 0, 0]  # x, y, theta

        # Matrices de gains de l'espace d'état
        self.K = np.array([[2.0, 2.0, 0.2], [0.2, 0.2, 2.0]])

        self.timer_period = 0.1  # seconds

        # Distance à maintenir entre les deux tortues
        self.FD = 1.0

        # Publishers and Subscriptions
        self.pose_turtle1_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback_driver, 10)
        self.pose_turtle_follower_sub = self.create_subscription(Pose, "/turtleFollower/pose", self.pose_callback_follower, 10)
        self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/turtleFollower/cmd_vel", 10)

        self.timer_follow = self.create_timer(self.timer_period, self.followCommand)
        self.get_logger().info("Shadowing started")

    def pose_callback_driver(self, pose: Pose):
        self.pose_turtle1 = [pose.x, pose.y, pose.theta, pose.linear_velocity, pose.angular_velocity]

    def pose_callback_follower(self, pose: Pose):
        self.pose_turtleFollower = [pose.x, pose.y, pose.theta]

    def send_velocity_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_turtle_follower_pub_.publish(msg)

    def followCommand(self):
        # Erreur d'état: position et angle
        error_x = self.pose_turtle1[0] - self.pose_turtleFollower[0]
        error_y = self.pose_turtle1[1] - self.pose_turtleFollower[1]

        # Angle relatif entre les deux tortues
        theta_relative = np.arctan2(error_y, error_x) - self.pose_turtleFollower[2]
        theta_relative = np.arctan2(np.sin(theta_relative), np.cos(theta_relative))  # Normalisation de l'angle

        # Ajuster les erreurs pour maintenir une distance cible FD
        error_x = error_x - self.FD * np.cos(theta_relative)
        error_y = error_y - self.FD * np.sin(theta_relative)

        # Erreur d'état
        error_position = np.array([error_x, error_y, theta_relative])

        # Calcul de la commande de contrôle: u = -K * e
        control_input = -np.dot(self.K, error_position)

        # Saturer les commandes
        #linear_speed = np.clip(control_input[0], -2.0, 2.0)
        #angular_speed = np.clip(control_input[1], -2.0, 2.0)

        linear_speed = control_input[0]
        angular_speed = control_input[1]

        # Log pour débogage
        self.get_logger().info(f"Errors: ex={error_x:.2f}, ey={error_y:.2f}, etheta={theta_relative:.2f}")
        self.get_logger().info(f"Commands: v={linear_speed:.2f}, w={angular_speed:.2f}")

        self.send_velocity_command(linear_speed, angular_speed)


def main(args=None):
    rclpy.init(args=args)
    node = node_cmdEtat()
    rclpy.spin(node)
    rclpy.shutdown()
