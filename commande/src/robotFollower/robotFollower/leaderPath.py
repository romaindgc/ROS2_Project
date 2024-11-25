#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class node_leaderPath(Node):

    def __init__(self):
        super().__init__('Follower')

        # Variables for the position
        self.pose_leader = [0, 0, 0, 0, 0]

        #Varibles for the trajectory
        self.trajectory = [[5,0],[5,-5],[-5,-5],[-5,0],[0,0]]

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

        self.Kp_angle = 1.0
        self.Ki_angle = 0.01
        self.Kd_angle = 0.2

        # Other parameters
        self.indexTraj = 0
        self.tolerance = 0.7 #For avoiding perpetual oscillations, we add a tolerance to our aimed distance
        self.timer_period = 0.1  # seconds

        # Publishers and Subscriptions
        self.pose_leader_sub = self.create_subscription(Odometry, "/model/vehicle_blue/odometry", self.pose_callback_leader, 10)
        self.cmd_vel_leader_pub_ = self.create_publisher(Twist, "/model/vehicle_blue/cmd_vel", 10)

        self.timer_follow = self.create_timer(self.timer_period, self.followCommand)
        self.get_logger().info("Race started")

    #Get the pose of the leader robot 
    def pose_callback_leader(self, msg: Odometry):
        quatertion_leader = msg.pose.pose.orientation
        angle_leader = quaternion_to_yaw(quatertion_leader.x, quatertion_leader.y,quatertion_leader.z, quatertion_leader.w)
        self.pose_leader = [msg.pose.pose.position.x, msg.pose.pose.position.y, angle_leader]
   
    #Send the velocity command to the leader robot
    def send_velocity_command(self, x, y, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(z)
        self.cmd_vel_leader_pub_.publish(msg)

    def followCommand(self):
        # Calculate errors
        self.error_distance = norme_euclidienne(self.trajectory[self.indexTraj], self.pose_leader)
        self.error_angle = np.arctan2(self.trajectory[self.indexTraj][1]- self.pose_leader[1], self.trajectory[self.indexTraj][0] - self.pose_leader[0]) - self.pose_leader[2]

        #Check if we reached the current point ie if it is neccesary to go to the next point
        if self.error_distance < self.tolerance and  self.error_distance > - self.tolerance :
            if self.indexTraj < len(self.trajectory)-1 :
                #Check if we finished the circuit
                self.indexTraj = self.indexTraj +1
            else :
                #If we ended the circuit we restart a lap 
                self.indexTraj = 0
            #We recompute the error with the new trajectory point
            self.error_distance = norme_euclidienne(self.trajectory[self.indexTraj], self.pose_leader)
            self.error_angle = np.arctan2(self.trajectory[self.indexTraj][1]- self.pose_leader[1], self.trajectory[self.indexTraj][0] - self.pose_leader[0]) - self.pose_leader[2]
        
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
        if self.error_distance > self.tolerance:
            self.send_velocity_command(linear_speed, 0, angle_speed)
        elif self.error_distance < - self.tolerance:
            self.send_velocity_command(-linear_speed, 0, -angle_speed)
        else:
            self.send_velocity_command(0, 0, 0)

        # Log pour débogage
        self.get_logger().info(f"error: index={self.indexTraj:.2f}, error_distance={self.error_distance:.2f}, error_angle={self.error_angle:.2f}")
        self.get_logger().info(f"Trajectoire : x={self.trajectory[self.indexTraj][0]:.2f}, y={self.trajectory[self.indexTraj][1]:.2f}")
        self.get_logger().info(f"Commande : v={linear_speed:.2f}, z={angle_speed:.2f}")


def norme_euclidienne(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def quaternion_to_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

def main(args=None):
    rclpy.init(args=args)
    node = node_leaderPath()
    rclpy.spin(node)
    rclpy.shutdown()
