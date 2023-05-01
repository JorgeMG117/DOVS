# from sympy.geometry import *
# from sympy import Ray
import math
import numpy as np

from matplotlib import pyplot as plt

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
    
    @staticmethod
    def hom(x):
        """
        Returns the homogeneous matrix of the position x, y, tita
        """
        H = np.array([[np.cos(x[2]), -np.sin(x[2]), x[0]],
                    [np.sin(x[2]), np.cos(x[2]), x[1]],
                    [0, 0, 1]])
        return H

    @staticmethod
    def loc(H):
        """
        Returns the position x, y, tita of the homogeneous matrix H
        """
        x = np.array([H[0, 2], H[1, 2], np.arctan2(H[1, 0], H[0, 0])])
        return x



class DynamicObstacleDOVS(ObjectDOVS):
    def __init__(self, obstacle, robot_radius, robot_location) -> None:
        # print("Posicion del robot visto desde el mundo")
        # print(robot_location)

        # print("Posicion del obstaculo visto desde el mundo")
        # print((obstacle.x, obstacle.y, obstacle.theta))
        # Get the obstacle position in the robot frame
        self.transform_robot = np.dot(np.linalg.inv(self.hom(robot_location)),self.hom((obstacle.x, obstacle.y, obstacle.theta)))
        obstacle_pos = self.loc(self.transform_robot)

        # El tamaño del objeto es el cuadrado que rodea al circulo
        self.radius = obstacle.radius + robot_radius

        super().__init__(obstacle.v, obstacle.w, obstacle_pos[0], obstacle_pos[1], obstacle_pos[2])
        # super().__init__(obstacle.v, obstacle.w, obstacle.x, obstacle.y, obstacle.theta)
    
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

        # TODO: Asi o con distintas clases
        if self.w != 0:
            radius = self.v/self.w
            center_x = self.x + radius * math.cos(self.theta + math.pi/2)
            center_y = self.y + radius * math.sin(self.theta + math.pi/2)

            radius_1 = ((center_x - x1) ** 2 + (center_y - y1) ** 2) ** 0.5
            radius_2 = ((center_x - x2) ** 2 + (center_y - y2) ** 2) ** 0.5  

            l1 = Circle(Point(center_x, center_y), radius_1)
            l2 = Circle(Point(center_x, center_y), radius_2)
            
        else:
            # Create a line using the point and angle
            ray_1 = Ray((0,0), angle=self.theta)
            l1 = Line(p1, slope=ray_1.slope)
            l2 = Line(p2, slope=ray_1.slope)
        
        return l1, l2
    

    def get_colision_points(self):
        """
        Devuelve los puntos del cuadrado circunscrito al obstaculo que seran los que choquen con los puntos de colision
        """
        value1 = self.loc(np.dot(self.transform_robot, self.hom((self.radius, self.radius, 0))))
        value2 = self.loc(np.dot(self.transform_robot, self.hom((-self.radius, -self.radius, 0))))
        
        return value1, value2
    
   
# class LinearObstacle(DynamicObstacleDOVS):
#     def compute_trajectory(self):
#         """
#         Returns the collision band of the obstacle(two trajectories)
#         [passBehind, passFront]
#         """

#         x1 = self.x + self.radius * math.cos(self.theta + math.pi/2)
#         y1 = self.y + self.radius * math.sin(self.theta + math.pi/2)
#         p1 = Point(x1, y1)

#         x2 = self.x + self.radius * math.cos(self.theta - math.pi/2)
#         y2 = self.y + self.radius * math.sin(self.theta - math.pi/2)
#         p2 = Point(x2, y2)

#         # Create a line using the point and angle
#         ray_1 = Ray((0,0), angle=self.theta)
#         l1 = Line(p1, slope=ray_1.slope)
#         l2 = Line(p2, slope=ray_1.slope)
        
#         return l1, l2

# class CircularObstacle(DynamicObstacleDOVS):
#     pass


class RobotDOVS(ObjectDOVS):
    def __init__(self, robot, velocity_window) -> None:

        goal_pos = self.loc(np.dot(np.linalg.inv(self.hom((robot.x, robot.y, robot.theta))),self.hom((robot.x_goal, robot.y_goal, robot.theta))))
        self.x_goal = goal_pos[0]
        self.y_goal = goal_pos[1]

        self.min_v = robot.min_v
        self.max_v = robot.max_v
        self.min_w = robot.min_w
        self.max_w = robot.max_w
        self.max_av = robot.max_av
        self.max_aw = robot.max_aw

        self.trajectory_max_radius = 10
        self.trajectory_step_radius = 2

        self.velocity_window = velocity_window

        super().__init__(robot.v, robot.w, 0, 0, 0)
        # super().__init__(robot.v, robot.w, robot.x, robot.y, robot.theta)
    
    def compute_trajectories(self):
        """
        Compute all the possible trajectories of the robot
        discretized set of feasible circular trajectories
        """

        trajectories = []

        for radius in range(-self.trajectory_max_radius, self.trajectory_max_radius, self.trajectory_step_radius):
            if radius == 0: continue
            
            center_x = self.x + radius * math.cos(self.theta + math.pi/2)
            center_y = self.y + radius * math.sin(self.theta + math.pi/2)
            c = Circle(Point(center_x, center_y), radius)
            
            trajectories.append(c)
        
        return trajectories
    

        