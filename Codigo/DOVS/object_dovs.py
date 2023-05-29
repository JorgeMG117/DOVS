# from sympy.geometry import *
# from sympy import Ray
import math
import numpy as np

# from matplotlib import pyplot as plt

from DOVS.geometry.trajectory import CircularTrajectory, LinearTrajectory, RobotTrajectory

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
    def __init__(self, obstacle, robot_radius, robot_location, max_distance) -> None:
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

        if self.theta < 0:
            self.obs_col_ahead = self.loc(np.dot(self.transform_robot, self.hom((self.radius, self.radius, 0))))
            self.obs_col_behind = self.loc(np.dot(self.transform_robot, self.hom((-self.radius, -self.radius, 0))))
        else:
            self.obs_col_behind = self.loc(np.dot(self.transform_robot, self.hom((-self.radius, self.radius, 0))))
            self.obs_col_ahead = self.loc(np.dot(self.transform_robot, self.hom((self.radius, -self.radius, 0))))

        self.trajectory = self.compute_trajectory(max_distance)

    
    def compute_trajectory(self, max_distance):
        """
        Returns the collision band of the obstacle(two trajectories)
        [passBehind, passFront]
        """

        x1 = self.x + self.radius * math.cos(self.theta + math.pi/2)
        y1 = self.y + self.radius * math.sin(self.theta + math.pi/2)

        x2 = self.x + self.radius * math.cos(self.theta - math.pi/2)
        y2 = self.y + self.radius * math.sin(self.theta - math.pi/2)

        # (x1, y1, _), (x2, y2, _) = self.get_colision_points()

        if self.w != 0:
            trajectory = CircularTrajectory((x1, y1), (x2, y2), (self.x, self.y, self.theta), (self.v, self.w))
        else:
            trajectory = LinearTrajectory((x1, y1), (x2, y2), self.theta, max_distance)
        
        return trajectory
    
    
    def get_colision_points(self):
        """
        Devuelve los puntos del cuadrado circunscrito al obstaculo que seran los que choquen con los puntos de colision
        """
        # self.obs_col_behind = self.loc(np.dot(self.transform_robot, self.hom((-self.radius, self.radius, 0))))
        # self.obs_col_ahead = self.loc(np.dot(self.transform_robot, self.hom((self.radius, -self.radius, 0))))
        
        return self.obs_col_behind, self.obs_col_ahead
    
    def inside_collision_band(self, position):
        is_inside = self.trajectory.contains_point(position[0], position[1])

        if is_inside:
            position_from_obstacle = self.loc(np.dot(np.linalg.inv(self.hom(self.get_location())),self.hom((position[0], position[1], position[2]))))
            #print(p1_from_obstacle[0])
            if position_from_obstacle[0] >= 0:
                return True
            else:
                return False
        else:
            return False


        p1, p2 = self.trajectory.get_value(x)

        # x1, y1 = p1
        # x2, y2 = p2
        
        # if (y1 > 0 and y2 < 0) or (y1 < 0 and y2 > 0):
        #     # Si un punto es negativo y otro positivo significa que el el robot esta en medio de la banda de colision
        #     # Ahora hay que ver que la orientacion del obstaculo sea hacia el robot
        #     p1_from_obstacle = self.loc(np.dot(np.linalg.inv(self.hom(self.get_location())),self.hom((x1, y1, 0))))
        #     #print(p1_from_obstacle[0])
        #     if p1_from_obstacle[0] >= 0:
        #         return True
        #     else:
        #         return False
        # else:
        #     return False 
    
    # def check_colision(self, x, y):
    #     """
    #     Given a point checks if its inside the cuadrado circunscrito
    #     """
    #     s = self.radius * math.sqrt(2)

    #     # Step 3: Determine the boundaries of the square
    #     min_x = self.x - s
    #     max_x = self.x + s
    #     min_y = self.y - s
    #     max_y = self.y + s

    #     # Step 4: Check if the point lies within the boundaries
    #     if min_x <= x <= max_x and min_y <= y <= max_y:
    #         return True
    #     else:
    #         return False
        
    
   
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

        self.trajectories = self.compute_trajectories()
    
    def compute_trajectories(self):
        """
        Compute all the possible trajectories of the robot
        discretized set of feasible circular trajectories
        """

        trajectories = []

        for radius in range(-self.trajectory_step_radius, -self.trajectory_max_radius-1, -self.trajectory_step_radius):
            # print(radius)
            trajectory = RobotTrajectory(radius, (self.x, self.y, self.theta))
            trajectories.append(trajectory)

        for radius in range(self.trajectory_max_radius, self.trajectory_step_radius-1, -self.trajectory_step_radius):
            # print(radius)
            trajectory = RobotTrajectory(radius, (self.x, self.y, self.theta))
            trajectories.append(trajectory)

        # for radius in range(-self.trajectory_max_radius, self.trajectory_max_radius, self.trajectory_step_radius):
        #     if radius == 0: continue
            
        #     trajectory = RobotTrajectory(radius, (self.x, self.y, self.theta))
        #     trajectories.append(trajectory)
        
        return trajectories
    
    # def get_trajectories(self):
    #     return self.trajectories


    # def normalize_speed(self, w, v, trajectory_radius):
    #     if v > self.max_v or w > self.max_w:
    #             v = self.max_v
    #             w = v/trajectory_radius
    #     elif v < self.min_v or w < self.min_w:
    #             v = self.min_v
    #             w = v/trajectory_radius
        
    #     return w, v
        
        
    
    def get_speed_goal(self):
        
        r = (pow(self.x_goal, 2) + pow(self.y_goal, 2))/(2*self.y_goal)
        v = self.max_v
        w = self.v/r 
        
        return w, v
    

        