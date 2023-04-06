import sympy
from sympy.geometry import *
import math
import matplotlib.pyplot as plt


class ObjectDOVS:
    def __init__(self, v, w, x, y, theta) -> None:
        self.v = v
        self.w = w
        self.x = x
        self.y = y
        self.theta = theta #Radians
    
    def show_object_dovs(self, id):
        print("Posicion " + str(id) + ": x = " + str(self.x) + ", y = " + str(self.y) + ", theta = " + str(self.theta))
        print("Velocidad " + str(id) + ": v =" + str(self.v) + ", w = " + str(self.w))

    def get_location(self):
        """
        Returns the location of the DOVS object
        """
        return self.x, self.y, self.theta
    
    def get_speed(self):
        """
        Returns the speed of the DOVS object
        """
        return self.v, self.w




class DynamicObstacleDOVS(ObjectDOVS):
    def __init__(self, obstacle, robot_radius) -> None:
        # Colision band

        # El tamaño del objeto es el cuadrado que rodea al circulo
        self.radius = obstacle.radius + robot_radius

        super().__init__(obstacle.v, obstacle.w, obstacle.x, obstacle.y, obstacle.theta)
    
    def compute_trajectory(self):
        """
        Returns the collision band of the obstacle(two trajectories)
        [passBehind, passFront]
        """

        x1 = self.x + self.radius * math.cos(self.theta + math.pi/2)
        y1 = self.y + self.radius * math.sin(self.theta + math.pi/2)
        p1 = Point(x1, y1)

        x2 = self.x + self.radius * math.cos(self.theta - math.pi/2)
        y2 = self.y + self.radius * math.sin(self.theta - math.pi/2)
        p2 = Point(x2, y2)

        # Create a line using the point and angle
        l1 = Line(p1, slope=self.theta)
        l2 = Line(p2, slope=self.theta)
        
        return l1, l2
    
   



class RobotDOVS(ObjectDOVS):
    def __init__(self, robot) -> None:

        self.x_goal = robot.x_goal
        self.y_goal = robot.y_goal
        self.min_v = robot.min_v
        self.max_v = robot.max_v
        self.min_w = robot.min_w
        self.max_w = robot.max_w
        self.max_av = robot.max_av
        self.max_aw = robot.max_aw

        self.trajectory_max_radius = 10
        self.trajectory_step_radius = 2

        super().__init__(robot.v, robot.w, robot.x, robot.y, robot.theta)
    
    def compute_trajectories(self):
        """
        Compute all the possible trajectories of the robot
        discretized set of feasible circular trajectories
        """

        trajectories = []

        for radius in range(-self.trajectory_max_radius, self.trajectory_max_radius, self.trajectory_step_radius):
            if radius == 0: continue
            # Creo que habria que tener en cuenta theta
            center_x = self.x + radius * math.cos(self.theta + math.pi/2)
            center_y = self.y + radius * math.sin(self.theta + math.pi/2)
            c = Circle(Point(center_x, center_y), radius)
            
            trajectories.append(c)
        
        return trajectories
    

        