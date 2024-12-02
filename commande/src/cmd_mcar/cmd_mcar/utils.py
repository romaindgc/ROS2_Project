import math
from turtlesim.msg import Pose
import random
from nav_msgs.msg import Odometry

def quaternion_to_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

# class to manipulate position more easily
class Pos:

    # define a pos with x, y and rotation
    def __init__(self, x = 0, y = 0, theta = 0):
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
    
    #return angle of line created by 2 points 
    def angle(self, p2):
        return math.atan2(p2.y - self.y, p2.x - self.x)

    def update(self, pose : Pose):
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta

    def update_gz(self, msg : Odometry, offset = 0):
        quatertion_turtleFollower = msg.pose.pose.orientation
        angle_turtleFollower = quaternion_to_yaw(quatertion_turtleFollower.x, quatertion_turtleFollower.y, quatertion_turtleFollower.z, quatertion_turtleFollower.w)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = angle_turtleFollower

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
        
    def __str__(self):
        return f"x : {self.x}, y : {self.y}, theta : {self.theta}"
        

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


class PID_cmd:

    def __init__(self, timer_period, tolerance=0.1, dist_target = 0, dist_gain = [1.0, 0.01, 0.2], angle_gain = [3.0, 0.01, 0.2]):

        # Variables for the PID
        self.error_distance_previous = 0
        self.error_angle_previous = 0

        self.integrated_distance = 0
        self.integrated_angle = 0

        # PID Gains
        [self.Kp_distance, self.Ki_distance, self.Kd_distance] = dist_gain

        [self.Kp_angle, self.Ki_angle, self.Kd_angle] = angle_gain

        # define distance to target
        self.dist_target = dist_target

        # tolerance to avoid perpetual oscillations
        self.tolerance = tolerance

        # time step value
        self.time_step = timer_period

    def commande(self, pos_act : Pos, pos_next : Pos):


        # New position calcuted with distance
        dif_pos = Pos(self.dist_target * math.cos(pos_next.theta), self.dist_target * math.sin(pos_next.theta), 0)
        target_pos = pos_next - dif_pos

        # Calculate errors
        error_distance = pos_act.distance(target_pos)
        error_angle = pos_act.angle(target_pos) - pos_act.theta


        # if near enough of where it should be then don't move
        if abs(error_distance) < self.tolerance:
            return(float(0), float(0))

        
        # Normalize angle in [-pi,pi] for ensuring that the turtle turns in the shortest sens
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

        # PID for distance
        self.integrated_distance += error_distance * self.time_step
        derivated_distance = (error_distance - self.error_distance_previous) / self.time_step
        linear_speed = self.Kp_distance * error_distance + self.Ki_distance * self.integrated_distance + self.Kd_distance * derivated_distance

        # PID for angle
        self.integrated_angle += error_angle * self.time_step
        derivated_angle = (error_angle - self.error_angle_previous) / self.time_step
        angle_speed = self.Kp_angle * error_angle + self.Ki_angle * self.integrated_angle + self.Kd_angle * derivated_angle

        # Update previous errors
        self.error_distance_previous = error_distance
        self.error_angle_previous = error_angle

        return (float(linear_speed),float(angle_speed) )



class Trajectory :

    def __init__(self, nb_point):
        #limit values for trajectory
        self.min_max = [[7.0,2.0], [7.0,2.0], [3.14,-3.14]]
        self.nb_point = nb_point

        self.point_act = nb_point 
        self.trajectory = [ Pos() for _ in range(nb_point)]

    def generate(self):
        for i in range(self.nb_point):
            rand_x = random.uniform(self.min_max[0][0], self.min_max[0][1])
            rand_y = random.uniform(self.min_max[1][0], self.min_max[1][1])
            rand_theta = random.uniform(self.min_max[2][0], self.min_max[2][1])

            self.trajectory[i] = Pos(rand_x, rand_y, rand_theta)
    
    def get_next_point(self):
        if self.point_act >= self.nb_point:
            self.point_act = 0
            self.generate()
        
        self.point_act += 1
        return(self.trajectory[self.point_act - 1])