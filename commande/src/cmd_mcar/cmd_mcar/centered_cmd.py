import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# class to manipulate position more easily
class Pos:

    # define a pos with x, y and rotation
    def __init__(self, x, y, theta = 0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    # return the a and b parameter of the perpendicular line of the point
    def eq_droite_perp(self):
        a = math.tan(self.theta + math.pi/2)
        b = self.y - a * self.x
        return Line(a, b)
    
    # return distance between 2 points
    def distance(self, p2):
        return math.sqrt((self.x - p2.x)**2 + (self.y - p2.y)**2)

    def update(self, pose : Pose):
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta


    def __add__(self, other):
        if isinstance(other, Pos):
            return Pos(self.x + other.x, self.y + other.y, self.theta + other.theta)
        else:
            raise ValueError("Pos value is needed")
        
    def __sub__(self, other):
        if isinstance(other, Pos):
            return Pos(self.x - other.x, self.y - other.y, self.theta - other.theta)
        else:
            raise ValueError("Pos value is needed")
        

# class to manipulate line more easily
class Line :

    def __init__(self, a, b):
        self.a = a
        self.b = b

    #return the point of intersection between two lines
    def find_intersection(self, line2):
        x = (line2.b - self.b) / (self.a - line2.a)
        y = self.a * x + self.b
        return Pos(x, y)

# return linear and angular velocity needed to go from pos1 to pos2 
def calc_traj(pos_act : Pos, pos_next : Pos) :

    # if near enough of where it should be the don't move
    if pos_act.distance(pos_next) < 0.1:
        return(0,0)

    # if the angles are the same then the robot just need to do a straight line
    if pos_act.theta == pos_next.theta:
        return (VMAX, 0)
    
    # else we need to calculate the radius of the circular arc and deduce the angular velocity needed 
    pcenter = pos_act.eq_droite_perp().find_intersection(pos_next.eq_droite_perp())
    r = pcenter.distance(pos_act)

    v_rot = - VMAX / r

    #we need to turn in the right direction
    if pos_act.theta - pos_next.theta <= 0 and pos_act.theta - pos_next.theta > math.pi:
        v_rot *= -1

    return ( float(VMAX),float(v_rot) )

VMAX = 1.0
class node_centered_cmd(Node):
    def __init__(self):
        super().__init__('node_centered_cmd')
        self.pose_turtle1 = Pos(0, 0)
        self.pose_turtleFollower = Pos(0, 0)
        
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.send_velocity_command)

        self.pose_turtle1_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_turtle1.update, 10)
        self.pose_turtle_folower_sub = self.create_subscription(Pose, "/turtleFollower/pose", self.pose_turtleFollower.update, 10)

        self.cmd_vel_turtle_follower_pub_ = self.create_publisher(Twist, "/turtleFollower/cmd_vel", 10)


    def send_velocity_command(self):
        msg = Twist()

        
        msg.linear.x, msg.angular.z = calc_traj(self.pose_turtleFollower, self.pose_turtleFollower + Pos(-1.0, -1.0, math.pi/2))

        self.cmd_vel_turtle_follower_pub_.publish(msg)
        





def main(args=None):
    rclpy.init(args=args)
    node = node_centered_cmd()
    rclpy.spin(node)
    rclpy.shutdown()









if __name__ == '__main__':
    main()
