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

# class Collisions:
#     def __init__(self) -> None:
#         self.x = 0
#         self.y = 0

#         self.angle = 0 #angle between robot and collision
#         self.arclenght = 0 #distance between robot and collision
#         self.distance_up = 0 #distance from the collision point to the object
#         self.distance_down = 0

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

        plotDOVS.plot_trajectories(robot_trajectories, obstacles_trajectory, collision_points_list)
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

                # TODO: con el angulo valdria no??
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
        


    def _collision_velocity_times_aux(self, collision_point, obs_col, trajectory_radius, v_obstacle, angle = None):
        x_col = float(collision_point[0])
        y_col = float(collision_point[1])

        x_obs_col = obs_col[0]
        y_obs_col = obs_col[1]

        distance = math.sqrt((x_col - x_obs_col)**2 + (y_col - y_obs_col)**2)

        t = distance/v_obstacle

        if angle == None:
            angle = np.arctan2(2*x_col*y_col, pow(x_col,2)-pow(y_col,2))

        w = angle/t
        v = trajectory_radius*w

        return (t, v, w)

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
        obs_col_behind, obs_col_ahead = obstacle.get_colision_points()
        v_object, _ = obstacle.get_speed()

        w_min = v_min = w_max = v_max = t_min = t_max = 0

        if collision_points[0] == None:
            # Calculo la velocidad maxima que puedo llevar
            # Pongo como minima el limite superior. No hay velocidad de escape
            (t_max, v_max, w_max) = self._collision_velocity_times_aux(collision_points[1], obs_col_behind, trajectory.radius, v_object)
            
            v_min = self.robot.max_v
            w_min = v_min/trajectory.radius

        elif collision_points[1] == None:
            # Calculo la velocidad maxima que puedo llevar
            # Pongo como minima el limite superior. No hay velocidad de escape
            (t_max, v_max, w_max) = self._collision_velocity_times_aux(collision_points[0], obs_col_behind, trajectory.radius, v_object)
            
            v_min = self.robot.max_v
            w_min = v_min/trajectory.radius
        else:
            idx_first = 0

            # Tenemos que ver cual de los dos puntos de colision es el que va a definir la velocidad maxima 
            x_col_1 = float(collision_points[0][0])
            y_col_1 = float(collision_points[0][1])

            x_col_2 = float(collision_points[1][0])
            y_col_2 = float(collision_points[1][1])


            angle_1 = np.arctan2(2*x_col_1*y_col_1, pow(x_col_1,2)-pow(y_col_1,2))
            angle_2 = np.arctan2(2*x_col_2*y_col_2, pow(x_col_2,2)-pow(y_col_2,2))
            
            #angle = np.arctan2(2*x*y, pow(x,2)-pow(y,2))
            angles = [angle_1, angle_2]

            if angle_1 > angle_2:
                idx_first = idx_first + 1
                
            (t_max, v_max, w_max) = self._collision_velocity_times_aux(collision_points[idx_first], obs_col_behind, trajectory.radius, v_object, angles[idx_first])

            (t_min, v_min, w_min) = self._collision_velocity_times_aux(collision_points[(idx_first+1)%2], obs_col_ahead, trajectory.radius, v_object, angles[(idx_first+1)%2])



        return (t_max, v_max, w_max), (t_min, v_min, w_min)

      
   


       
    def _create_DOVS(self, velocity_time_space):
        """
        Dada la lista de velocidades crea el dovs de manera que se le puedan añadir mas dovs
        velocity_time_space = [(t1,v1,w1),(t2,v2,w2),(t3,v3,w3),(t4,v4,w4)]
        """
        print(velocity_time_space)
        velocity_time_space = [(1, 2, 3),(2,4,4),(1,5,2),(1,7,7),(1,1,1),(2,8,8)]
        t, v, w = zip(*velocity_time_space)

        # v.sort()
        # w.sort()

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

