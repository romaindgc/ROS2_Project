#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import numpy as np
import math
import random

#On reprend le code du node robot folower et on l'addapte pour avoir qu'une seul tortue (turtle folower) qui suis une trajectoire à la place de turtle 1
class node_follow_trajectoire(Node):

    def __init__(self):
        super().__init__('Follower')

        # Variables for the position
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
        self.FD = 0.0  # Target distance
        self.timer_period = 0.1  # seconds

        # Publishers and Subscriptions
        #self.pose_trajectoire_sub = self.create_subscription(Odometry, "/model/vehicle_blue/odometry", self.pose_callback_driver, 10)
        #self.pose_turtle_follower_sub = self.create_subscription(Odometry, "/model/vehicle_green/odometry", self.pose_callback_follower, 10)
        #self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/model/vehicle_green/cmd_vel", 10)
        self.pose_turtle_follower_sub = self.create_subscription(Pose, "/turtleFollower/pose", self.pose_callback_follower, 10)
        self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/turtleFollower/cmd_vel", 10)

        self.timer_follow = self.create_timer(self.timer_period, self.followCommand)
        self.get_logger().info("Shadowing started")

        #initialisation de la trajectoire
        pose_init=self.pose_turtleFollower #vérifier que la pose init soit bien la pose actuel de turtle folower avant commencemant de la trajectoire
        nombre_de_points=5
        self.min_max=[[7.0,2.0],[7.0,2.0],[3.14,-3.14],[2.0,0.5],[2.0,0.5],]

        self.pose_trajectoire=[[5,5,0,1,1],[5,5,0,1,1],[5,5,0,1,1],[5,5,0,1,1],[5,5,0,1,1]]
        self.indice_trajectoire=4

    #def pose_callback_driver(self, msg: Odometry):
    #    quatertion_turtle1 = msg.pose.pose.orientation
    #    angle_turtle1 = quaternion_to_yaw(quatertion_turtle1.x, quatertion_turtle1.y,quatertion_turtle1.z, quatertion_turtle1.w)
    #    self.pose_turtle1 = [msg.pose.pose.position.x, msg.pose.pose.position.y, angle_turtle1]

    #def pose_callback_follower(self, msg: Odometry):
    #    quatertion_turtleFollower = msg.pose.pose.orientation
    #    angle_turtleFollower = quaternion_to_yaw(quatertion_turtleFollower.x, quatertion_turtleFollower.y, quatertion_turtleFollower.z, quatertion_turtleFollower.w)
    #    self.pose_turtleFollower = [msg.pose.pose.position.x, msg.pose.pose.position.y, angle_turtleFollower]     
    def pose_callback_follower(self, pose: Pose):
        self.pose_turtleFollower = [pose.x, pose.y, pose.theta]

    def send_velocity_command(self, x, y, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(z)
        self.cmd_vel_turtle_follower_pub_.publish(msg)

    def generate_trajectory(self):
        self.pose_trajectoire[0]=[self.pose_turtleFollower[0],self.pose_turtleFollower[1],self.pose_turtleFollower[2],0,0]
        for i in range(4):
            self.pose_trajectoire[i+1][0]=random.uniform(self.min_max[0][0],self.min_max[0][1])
            self.pose_trajectoire[i+1][1]=random.uniform(self.min_max[1][0],self.min_max[1][1])
            self.pose_trajectoire[i+1][2]=random.uniform(self.min_max[2][0],self.min_max[2][1])
            self.pose_trajectoire[i+1][3]=random.uniform(self.min_max[3][0],self.min_max[3][1])
            self.pose_trajectoire[i+1][4]=random.uniform(self.min_max[4][0],self.min_max[4][1])

    def followCommand(self):
        # Calculate errors
        self.error_distance = norme_euclidienne(self.pose_trajectoire[self.indice_trajectoire], self.pose_turtleFollower)
        self.error_angle = np.arctan2(self.pose_trajectoire[self.indice_trajectoire][1] - self.pose_turtleFollower[1], self.pose_trajectoire[self.indice_trajectoire][0] - self.pose_turtleFollower[0]) - self.pose_turtleFollower[2]
        
        #Si on est suffisement proche du point sible alors on passe au point suivant si la trajectoire n'est pas fini sinon on en créer une nouvelle
        if self.error_distance < 0.5 : #contrainte de proximité sur la distance pas sur l'angle
            self.indice_trajectoire+=1
            if self.indice_trajectoire >4:
                self.indice_trajectoire=0
                self.generate_trajectory()
            self.error_distance = norme_euclidienne(self.pose_trajectoire[self.indice_trajectoire], self.pose_turtleFollower)
            self.error_angle = np.arctan2(self.pose_trajectoire[self.indice_trajectoire][1] - self.pose_turtleFollower[1], self.pose_trajectoire[self.indice_trajectoire][0] - self.pose_turtleFollower[0]) - self.pose_turtleFollower[2]
        
          
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
        
        # Log pour débogage
        self.get_logger().info(f"indice_trajectoire: ex={self.indice_trajectoire:.2f}, error_distance={self.error_distance:.2f}, error_angle={self.error_angle:.2f}")
        self.get_logger().info(f"poseTrajectoire{self.pose_trajectoire[self.indice_trajectoire][0]:.2f}")

def norme_euclidienne(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

#def quaternion_to_yaw(qx, qy, qz, qw):
#    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

def main(args=None):
    rclpy.init(args=args)
    node = node_follow_trajectoire()
    rclpy.spin(node)
    rclpy.shutdown()