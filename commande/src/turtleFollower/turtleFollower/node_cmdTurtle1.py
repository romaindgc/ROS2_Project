import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class node_cmdTurtle1(Node):

    def __init__(self):
        super().__init__('node_cmdTurtle1')
        self.pose_turtle1=[0, 0]
        self.pose_turtleFollower=[0, 0]



        self.pose_turtle1_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback_T1, 10)
        self.pose_turtle_follower_sub = self.create_subscription(Pose, "/turtleFollower/pose", self.pose_callback_Tf, 10)

        self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/turtleFollower/cmd_vel", 10)

        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.send_velocity_command)
        self.timer = self.create_timer(timer_period, self.print_pose_callback)
        self.get_logger().info("cmdTurtlefolower node has been started")
        self.get_logger().info("position turtle1 : x=" + str(self.pose_turtle1[0]) + " y="+str(self.pose_turtle1[1]))
        self.get_logger().info("position turtleFolower: x=" + str(self.pose_turtleFollower[0]) + " y="+str(self.pose_turtleFollower[1]))
    
    def pose_callback_T1(self, pose:Pose):
        cmd= Twist()
        self.pose_turtle1[0]=pose.x
        self.pose_turtle1[1]=pose.y
      
    def pose_callback_Tf(self, pose:Pose):
        self.pose_turtleFollower[0]=pose.x
        self.pose_turtleFollower[1]=pose.y
     
    def print_pose_callback(self):
        self.get_logger().info("position turtle1 : x=" + str(self.pose_turtle1[0]) + " y="+str(self.pose_turtle1[1]))
        self.get_logger().info("position turtleFollower: x=" + str(self.pose_turtleFollower[0]) + " y="+str(self.pose_turtleFollower[1]))


    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        #msg.linear.y = 1.0
        msg.angular.z = 1.0
        self.cmd_vel_turtle_follower_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = node_cmdTurtle1()
    rclpy.spin(node)
    rclpy.shutdown()
