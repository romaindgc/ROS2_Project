import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import time


class node_cmdTurtle1(Node):

    def __init__(self):
        super().__init__('Follower')
        self.pose_turtle1=[0, 0, 0, 0, 0]
        self.pose_turtleFollower=[0, 0, 0, 0, 0]
        self.rotationSpeed = 3.0 #We use a constant rotation speed for adjusting the angle
        self.FD = 1.0 #Distance that we aim to have between the two robots
        self.linearVelocity = 1.0 #Linear velocity of the follower
        self.Kp = 6.0 #Proportionnal coefficient

        self.pose_turtle1_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback_driver, 10) #subscription to the pose topic of the driver
        self.pose_turtle_follower_sub = self.create_subscription(Pose, "/turtleFollower/pose", self.pose_callback_follower, 10) #subcription to the pose topic of the follower

        self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/turtleFollower/cmd_vel", 10) #Creation of the publisher to adjust the pose of the follower

        timer_period = 0.5  # seconds
        self.timer_follow = self.create_timer(timer_period, self.followCommand)
        self.timer_print = self.create_timer(timer_period, self.print_pose_callback)
        self.get_logger().info("Shadowing started")
    
    def pose_callback_driver(self, pose:Pose):
        cmd= Twist()
        self.pose_turtle1[0] = pose.x
        self.pose_turtle1[1] = pose.y
        self.pose_turtle1[2] = pose.theta
        self.pose_turtle1[3] = pose.linear_velocity
        self.pose_turtle1[4] = pose.angular_velocity
      
    def pose_callback_follower(self, pose:Pose):
        self.pose_turtleFollower[0]=pose.x
        self.pose_turtleFollower[1]=pose.y
     
    def print_pose_callback(self):
        self.get_logger().info("position turtle1 : x=" + str(self.pose_turtle1[0]) + " y="+str(self.pose_turtle1[1]))
        self.get_logger().info("position turtleFollower: x=" + str(self.pose_turtleFollower[0]) + " y="+str(self.pose_turtleFollower[1]))


    def send_velocity_command(self, x , z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.angular.z = float(z)
        self.cmd_vel_turtle_follower_pub_.publish(msg)

    # def followCommand(self) :
    #     d = abs(self.pose_turtle1[0]-self.pose_turtleFollower[0]) #compute the distance along the X axes
    #     c = abs(self.pose_turtle1[1]-self.pose_turtleFollower[1]) #compute the distance along the Y axes

    #     theta = np.arctan2(c,d) #compute the angle/orientation between the two robots

    #     rotation_time = (theta/self.rotationSpeed) #compute the time while we are turning
    #     self.send_velocity_command(0,self.rotationSpeed)
    #     time.sleep(rotation_time)
    #     self.send_velocity_command(0,0)
    #     #At this point, the orientation of the follower should be good
    #     #We have to go straight forward

    #     error = abs(self.FD - norme_euclidienne(self.pose_turtle1, self.pose_turtleFollower)) #the distance is the error - FD because we want a distance of FD
    #     new_speed = error * self.Kp * self.linearVelocity

    #     self.send_velocity_command(new_speed,0)
    

    def followCommand(self):
        
        #Matrix creations
        #Vector q point b
        qpb = np.zeros([1,3])
        qpb[0,0] = self.pose_turtle1[3]
        qpb[0,2] = self.pose_turtle1[4]


        #Rotational matrice between the driver and the follower
        Rot = np.zeros[[3,3]]
        Rot[0,0] = np.cos(self.pose_turtle1[2])
        Rot[0,1] = -np.sin(self.pose_turtle1[2])
        Rot[1,0] = np.sin(self.pose_turtle1[2])
        Rot[1,1] = np.cos(self.pose_turtle1[2])
        Rot[2,2] = 1

        #Compute the output velocity vector
        vv = np.linalg.inv(Rot) @ qpb

        #vv[0]= velocity along x
        #vv[1]= velocity along y
        #vv[2]= angular velocity w

def norme_euclidienne(a,b) :
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)



def main(args=None):
    rclpy.init(args=args)
    node = node_cmdTurtle1()
    rclpy.spin(node)
    rclpy.shutdown()
