import sympy
from sympy.geometry import *
import math
import matplotlib.pyplot as plt

import numpy as np

from sympy import nsolve, Symbol, symbols

from DOVS.object_dovs import DynamicObstacleDOVS, RobotDOVS
from DOVS.plot_dovs import PlotDOVS

"""
https://matplotlib.org/stable/api/_as_gen/matplotlib.patches.Polygon.html
Buscar funcion que dado un poligono ver si velocidades pertence a ese poligono
https://www.matecdev.com/posts/shapely-polygon-from-points.html
"""

class DOVS:
    def __init__(self, robot, obstacles, timestep) -> None:
        self.robot = RobotDOVS(robot)
        self.obstacles = list(map(lambda obstacle: DynamicObstacleDOVS(obstacle, robot.radius, (robot.x, robot.y, robot.theta)), obstacles))
        self.timestep = timestep


    def compute_DOVS(self):
        """
        Compute the DOVS algorithm

        This function should be call every timestep
        """

        plotDOVS = PlotDOVS(self.robot, self.obstacles)
        
        # Obtain all pasible trajectories of robot
        robot_trajectories = self.robot.compute_trajectories()
        
        obstacles_trajectory = []
        for obstacle in self.obstacles:
            # Compute the trajectory of the obstacle
            obstacle_trajectory = obstacle.compute_trajectory()

            obstacles_trajectory.append(obstacle_trajectory)
        
            velocity_time_space = []
            collision_points_list = []
            for trajectory in robot_trajectories:
                collision_points = self._compute_collision_points(trajectory, obstacle_trajectory)
                # print("Collision points")
                # print(collision_points)
                collision_points_list.append(collision_points)

                velocity_time = self._collision_velocity_times(obstacle, collision_points, trajectory)
                velocity_time_space.append(velocity_time)
            
            

            # dovs = self.create_DOVS(velocity_time_space)#Calculamos el dovs para ese objeto

        plotDOVS.plot_trajectories(robot_trajectories, obstacles_trajectory, collision_points_list)
        #     self.append_DOVS(list_dovs, dovs)#Añadimos el dovs calculado al dovs total, geometrically merged

        
        # self.plot_DOVS(velocity_time_space)
        
        
    # TODO: It doesn't selects the right collision point
    def _select_right_collision_point(self, collision_points, trajectory_radius, robot_position):
        """
        Select the right collision point of the intersection of the robot trajectory and the obstacle trajectory
        """
        if collision_points:
            if len(collision_points) == 1:
                return collision_points[0]
            else:
                # Devolver el punto de colision mas cercano a la trayectoria del robot
                # Calculo longitud arco desde el robot 

                # Primer punto de colision
                # d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
                # θ = 2 * arctan(d/2r)
                # L = r * θ
                d = math.sqrt((collision_points[0][0] - robot_position[0])**2 + (collision_points[0][1] - robot_position[1])**2)
                theta = 2 * math.atan(d / (2 * trajectory_radius))
                arclength_1 = trajectory_radius * theta

                # Segundo punto de colision
                d = math.sqrt((collision_points[1][0] - robot_position[0])**2 + (collision_points[1][1] - robot_position[1])**2)
                theta = 2 * math.atan(d / (2 * trajectory_radius))
                arclength_2 = trajectory_radius * theta

                if arclength_1 <= arclength_2:
                    return collision_points[0]
                else:
                    return collision_points[1]
        else:
            return None


    def _compute_collision_points(self, trajectory, collision_band):
        """
        Compute the collision points of the trajectory with the collision band
        returns (passBehindCollisionPoint, passFrontCollisionPoint)
        :return: returns a tuple with two values, if there is an intersection the value will be a Point2D, if not it will be None
        """
        
        intersection_1 = trajectory.intersection(collision_band[0])
        intersection_2 = trajectory.intersection(collision_band[1])

        return self._select_right_collision_point(intersection_1, trajectory.radius, self.robot.get_location()), self._select_right_collision_point(intersection_2, trajectory.radius, self.robot.get_location())
        

    # TODO: Implement
    def _collision_velocity_times(self, obstacle, collision_points, trajectory):
        """
        APENDIX B
        Devuelve tiempo y velocidades minimas y maximas que debe llevar le robot para no colisionar con el obstaculo
        Se llama a esta funcion con cada una de las trayectorias del robot y con cada uno de los obstaculos
        Si no hay puntos de colision no hay limite de velocidad
        :param obstacle: obstaculo con el que se va a comprobar la colision
        :param collision_points: puntos de colision de la trayectoria del robot con el obstaculo, hay un maximo de dos colisiones
        :param trajectory: trayectoria circular del robot
        :return ((t1, v_max, w_max),(t2, v_min, w_min))
        """
        x_object, y_object, theta_object = obstacle.get_location()
        v_object, w_object = obstacle.get_speed()
        velocity_times = []


        for collision in collision_points:
            if collision != None:
                x_collision, y_collision = collision.coordinates#TODO: No utilizo para nada las colisiones, para que las he calculado entonces?

                # t = Symbol('t')

                # TODO: No entiendo bien porque he calculado los puntos de colision si no los utilizo en la formula para calcular velocidades
                # x_collision = x_object + v_object * math.cos(theta_object) * t_i
                # y_collision = y_object + v_object * math.sin(theta_object) * t_i
                # trajectory.radius ** 2 = x_collision ** 2 + (y_collision - trajectory.y) ** 2

                a = v_object ** 2

                # # TODO: Es el angulo el de theta_object?
                # b = 2 * v_object * (x_object * math.cos(theta_object) + y_object * math.sin(theta_object)) - 2 * v_object * math.sin(theta_object) * trajectory.y
                
                # c = x_object ** 2 + y_object ** 2 + trajectory.y ** 2 - 2 * y_object * trajectory.y - trajectory.radius ** 2
                # times = np.roots([a, b, c])

                # for t in times:
                #     w = trajectory.y / t #TODO: Esto no esta bien
                #     v = trajectory.radius * w
                #     velocity_times.append((t, v, w))
                # # x, y, z = symbols('x y z ')

                # # eq1 = 3 * t1**2 - 2 * x2**2 - 1
                # # eq2 = x1**2 - 2 * x1 + x2**2 + 2 * x2 - 8
                # # print(nsolve((f1, f2), (x1, x2), (-1, 1)))
                # # sol = solve(eqs, (x, y, z))
        
        return velocity_times
   


       
    def create_DOVS(velocity_time_space):
        """
        Dada la lista de velocidades crea el dovs de manera que se le puedan añadir mas dovs
        """
        pass



