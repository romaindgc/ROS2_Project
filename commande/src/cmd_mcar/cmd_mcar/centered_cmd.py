import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from .utils import *

class node_centered_cmd(Node):

    def __init__(self):
        super().__init__('node_centered_cmd')
        self.pose_turtle1 = Pos(0, 0)
        self.pose_turtleFollower1 = Pos(0, 0)
        self.pose_turtleFollower2 = Pos(0, 0)


        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.send_velocity_command)


        self.pose_turtle1_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_turtle1.update, 10)
        self.pose_turtle_folower1_sub = self.create_subscription(Pose, "/turtleFollower1/pose", self.pose_turtleFollower1.update, 10)
        self.pose_turtle_folower2_sub = self.create_subscription(Pose, "/turtleFollower2/pose", self.pose_turtleFollower2.update, 10)


        self.cmd_vel_turtle_follower1_pub_ = self.create_publisher(Twist, "/turtleFollower1/cmd_vel", 10)
        self.cmd_vel_turtle_follower2_pub_ = self.create_publisher(Twist, "/turtleFollower2/cmd_vel", 10)


        self.turtleFol1_cmd = PID_cmd(self.timer_period)
        self.turtleFol2_cmd = PID_cmd(self.timer_period)

        self.trajectory = Trajectory(4)
        self.target_pos = self.trajectory.get_next_point()



    def send_velocity_command(self):

        distance_center = 1
        diff_pos = Pos(- distance_center * math.sin(self.pose_turtle1.theta), distance_center * math.cos(self.pose_turtle1.theta))
  
        msg = [Twist(), Twist()]
        
        msg[0].linear.x, msg[0].angular.z = self.turtleFol1_cmd.commande(self.pose_turtleFollower1, self.target_pos + diff_pos)
        msg[1].linear.x, msg[1].angular.z = self.turtleFol2_cmd.commande(self.pose_turtleFollower2, self.target_pos - diff_pos)

        cmd_tt = 0
        for i in range(len(msg)):
            cmd_tt += msg[i].linear.x + msg[i].angular.z
        
        if cmd_tt == 0:
            self.target_pos = self.trajectory.get_next_point()

        self.cmd_vel_turtle_follower1_pub_.publish(msg[0])
        self.cmd_vel_turtle_follower2_pub_.publish(msg[1])



def main(args=None):
    rclpy.init(args=args)
    node = node_centered_cmd()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
