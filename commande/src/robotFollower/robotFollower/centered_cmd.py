import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from .utils import *

class node_centered_cmd(Node):

    def __init__(self):
        super().__init__('node_centered_cmd')

        # Initialisation of the positions of the robots
        self.robot1_pos = Pos()
        self.robot2_pos = Pos()
        self.robot2_pos.set_offset(Pos(-4))

        # Creation of the timer 
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.send_velocity_command)

        # Subscrition
        self.pose_robot1_sub = self.create_subscription(Odometry, "/model/vehicle_blue/odometry", self.robot1_pos.update_gz, 10)
        self.pose_robot2_sub = self.create_subscription(Odometry, "/model/vehicle_green/odometry", self.robot2_pos.update_gz, 10)
        
        # Publisher
        self.cmd_vel_robot1_pub_ = self.create_publisher(Twist, "/model/vehicle_blue/cmd_vel", 10)
        self.cmd_vel_robot2_pub_ = self.create_publisher(Twist, "/model/vehicle_green/cmd_vel", 10)

        # create the class with the commands of PID
        self.turtleFol1_cmd = PID_cmd(self.timer_period, tolerance=0.4, dist_gain=[1.0,0.01,0.2], angle_gain=[1.0,0.01,0.2])
        self.turtleFol2_cmd = PID_cmd(self.timer_period, tolerance=0.4, dist_gain=[1.0,0.01,0.2], angle_gain=[1.0,0.01,0.2])

        # Create the class with trajectory
        self.trajectory = Trajectory(mode="square")
        self.target_pos = self.trajectory.get_next_point()


    # Function that will send command to control the robots 
    def send_velocity_command(self):

        # Calculate the x and y difference needed for the point to be at a certain distance to the left or right
        distance_center = 2
        diff_pos = Pos(- distance_center * math.sin(self.target_pos.theta), distance_center * math.cos(self.target_pos.theta))
  
        msg = [Twist(), Twist()]
        
        # get the calculated values of linear and angular speed 
        msg[0].linear.x, msg[0].angular.z = self.turtleFol1_cmd.commande(self.robot1_pos, self.target_pos + diff_pos)
        msg[1].linear.x, msg[1].angular.z = self.turtleFol2_cmd.commande(self.robot2_pos, self.target_pos - diff_pos)

        # if both are near enough of the point, then go to next point
        cmd_tt = 0
        tolerance = 0.3
        for i in range(len(msg)):
            cmd_tt += abs(msg[i].linear.x) + abs(msg[i].angular.z)
        
        if cmd_tt <= tolerance:
            self.target_pos = self.trajectory.get_next_point()


        self.cmd_vel_robot1_pub_.publish(msg[0])
        self.cmd_vel_robot2_pub_.publish(msg[1])



def main(args=None):
    rclpy.init(args=args)
    node = node_centered_cmd()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
