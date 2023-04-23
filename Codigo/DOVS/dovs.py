import sympy
from sympy.geometry import *
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import numpy as np

from sympy import nsolve, Symbol, symbols

from DOVS.object_dovs import DynamicObstacleDOVS, RobotDOVS
from DOVS.plot_dovs import PlotDOVS

"""
https://matplotlib.org/stable/api/_as_gen/matplotlib.patches.Polygon.html
Buscar funcion que dado un poligono ver si velocidades pertence a ese poligono
https://www.matecdev.com/posts/shapely-polygon-from-points.html
"""

class Collisions:
    def __init__(self) -> None:
        self.x = 0
        self.y = 0

        self.angle = 0 #angle between robot and collision
        self.arclenght = 0 #distance between robot and collision
        self.distance_up = 0 #distance from the collision point to the object
        self.distance_down = 0

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
            

            dovs = self._create_DOVS(velocity_time_space)#Calculamos el dovs para ese objeto

        # plotDOVS.plot_trajectories(robot_trajectories, obstacles_trajectory, collision_points_list)
        #     self.append_DOVS(list_dovs, dovs)#Añadimos el dovs calculado al dovs total, geometrically merged

        
        # plotDOVS.plot_DOVS(dovs)
        
    
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
        

    def _collision_velocity_times(self, obstacle, collision_points, trajectory):
        """
        APENDIX B
        Devuelve tiempo y velocidades minimas y maximas que debe llevar le robot para no colisionar con el obstaculo
        Se llama a esta funcion con cada una de las trayectorias del robot y con cada uno de los obstaculos
        Si no hay puntos de colision no hay limite de velocidad
        :param obstacle: obstaculo con el que se va a comprobar la colision
        :param collision_points: puntos de colision de la trayectoria del robot con el obstaculo, hay un maximo de dos colisiones
        :param trajectory: trayectoria circular del robot
        :return ((t1, v_max, w_max),(t2, v_min, w_min)). velocity_time_space
        """
        x_object, y_object, theta_object = obstacle.get_location()
        v_object, _ = obstacle.get_speed()
        velocity_times = []

        # if collision_points[0] == None:
        #     # Calculo la velocidad maxima que puedo llevar
        #     # Pongo como minima el limite superior. No hay velocidad de escape

        #     # El punto desde el que hay calcular distancia Esta a la izq del objeto
        #     x_obj = obstacle.x + obstacle.radius * math.cos(obstacle.theta - math.pi/2)
        #     y_obj = obstacle.y + obstacle.radius * math.sin(obstacle.theta - math.pi/2)

        #     x_col = collision_points[1][0]
        #     y_col = collision_points[1][1]

        #     distance = math.sqrt((x_col - x_obj)**2 + (y_col - y_obj)**2)

        #     t = distance/v_object

        #     x_robot, y_robot, _ = self.robot.get_location()
        #     # d = math.sqrt((x_col - x_robot)**2 + (y_col - y_robot)**2)
        #     # theta = 2 * math.atan(d / (2 * trajectory.radius))
        #     # arclength = trajectory.radius * theta

        #     angle = np.arctan2(2*x_object*y_object, pow(x_object,2)-pow(y_object,2))
        #     r = radio trayectoria robo

        #     w = angle/t
        #     v = r*w
        # elif collision_points[1] == None:
        #     # Calculo la velocidad maxima que puedo llevar
        #     # Pongo como minima el limite superior. No hay velocidad de escape
        #     pass
        # else:
        #     idx_first = 0 # Tenemos que saber que punto de colision es el que corresponde al passBehind
        #     # Puntos del cuadrado circunscrito al objeto#TODO
        #     x1 = self.x + self.radius * math.cos(self.theta + math.pi/2)
        #     y1 = self.y + self.radius * math.sin(self.theta + math.pi/2)

        #     x2 = self.x + self.radius * math.cos(self.theta - math.pi/2)
        #     y2 = self.y + self.radius * math.sin(self.theta - math.pi/2)
            
        #     # Tenemos que ver cual de los dos puntos de colision es el que va a definir la velocidad maxima 
        #     x_robot, y_robot, th_robot = self.robot.get_location()

        #     d = math.sqrt((collision_points[0][0] - x_robot)**2 + (collision_points[0][1] - y_robot)**2)
        #     theta = 2 * math.atan(d / (2 * trajectory.radius))
        #     arclength_1 = trajectory.radius * theta

        #     # Segundo punto de colision
        #     d = math.sqrt((collision_points[1][0] - x_robot)**2 + (collision_points[1][1] - y_robot)**2)
        #     theta = 2 * math.atan(d / (2 * trajectory.radius))
        #     arclength_2 = trajectory.radius * theta

        #     if arclength_1 > arclength_2:
        #         idx_first = 1

        #     # First speed
        #     #t_col1 = distancia(p_1, collision_points[idx_first])/v_object

        return velocity_times

        # La linea con menos angulo es la primera en colisionar
        # Calcular angulo entre el robot y el punto de colision
        # Seleccionamos la primera como el que menos angulo tenga

      
   


       
    def _create_DOVS(self, velocity_time_space):
        """
        Dada la lista de velocidades crea el dovs de manera que se le puedan añadir mas dovs
        velocity_time_space = [(t1,v1,w1),(t2,v2,w2),(t3,v3,w3),(t4,v4,w4)]
        """
        velocity_time_space = [(1, 2, 3),(2,4,4),(1,5,2),(1,7,7),(1,1,1),(2,8,8)]
        t, v, w = zip(*velocity_time_space)

        v.sort()
        w.sort()

        #t, v, w = []

        # # Extract x and y coordinates from points
        # x = [point[1] for point in velocity_time_space]
        # y = [point[2] for point in velocity_time_space]

        # # Add first point to end of lists to close the polygon
        # x.append(x[0])
        # y.append(y[0])

        # # Plot polygon
        # plt.plot(x, y)
        # plt.show()
        #is_inside = polygon.contains_point(point)



        # vertices = [(0, 0), (0, 1), (1, 1), (1, 0.5), (0.5, 0)]

        # # Create a polygon object with the given vertices
        # polygon = patches.Polygon(vertices, facecolor='red', edgecolor='black')

        # # Create a figure and axis object
        # fig, ax = plt.subplots()

        # # Add the polygon to the axis
        # ax.add_patch(polygon)

        # # Set the limits of the axis
        # ax.set_xlim([0, 1])
        # ax.set_ylim([0, 1])

        # # Show the plot
        # plt.show()

