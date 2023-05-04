# import sympy
# from sympy.geometry import *
import math
from matplotlib import pyplot as plt
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches

import numpy as np

# from sympy import nsolve, Symbol, symbols
# from sympy import Polygon, plot
from DOVS.geometry.velocity_window import VelocityWindow


from DOVS.object_dovs import DynamicObstacleDOVS, RobotDOVS
from DOVS.plot_dovs import PlotDOVS

from DOVS.geometry.trajectory import intersection

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

#TODO: Mirar que hacer cuando el obstaculo esta mirando hacia un lado y no deberia chocar con nada
class DOVS:
    def __init__(self, robot, obstacles, timestep) -> None:
        velocity_window = VelocityWindow(robot, timestep)

        self.robot = RobotDOVS(robot, velocity_window)
        self.obstacles = list(map(lambda obstacle: DynamicObstacleDOVS(obstacle, robot.radius, (robot.x, robot.y, robot.theta)), obstacles))
        self.timestep = timestep

        #El poligono de velocidades prohibidas, #dynamic_object_velocity


    def compute_DOVS(self):
        """
        Compute the DOVS algorithm

        This function should be call every timestep
        """

        

        # plotDOVS.prueba()
        
        # Obtain all pasible trajectories of robot
        robot_trajectories = self.robot.compute_trajectories()
        
        obstacles_trajectory = []
        list_dovs = []
        collision_points_list = []
        for obstacle in self.obstacles:
            # Compute the trajectory of the obstacle
            obstacle_trajectory = obstacle.compute_trajectory()
            obstacles_trajectory.append(obstacle_trajectory)
        
            velocity_time_space = []
            #robot_trajectories = robot_trajectories[6:]
            for trajectory in robot_trajectories:
                
                # Compute collision points of an obstacle for every robot trajectory
                collision_points = self._compute_collision_points(trajectory, obstacle_trajectory)
                collision_points_list.append(collision_points)

                velocity_time = self._collision_velocity_times(obstacle, collision_points, trajectory)
                velocity_time_space.append(velocity_time)
                
            dovs = self._create_DOVS(velocity_time_space)#Calculamos el dovs para ese objeto
            list_dovs.append(dovs)

        dovs = self._combine_DOVS(list_dovs)#Añadimos el dovs calculado al dovs total, geometrically merged

        plotDOVS = PlotDOVS(self.robot, self.obstacles)
        #plotDOVS.plot_trajectories(robot_trajectories, obstacles_trajectory, collision_points_list)
        plotDOVS.plot_DOVS(velocity_time_space)
        plt.show()
        
       
        
        # plotDOVS.plot_DOVS(dovs)
        plt.show()

        return self._choose_speed()
        
    
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


    def _compute_collision_points(self, trajectory, obstacle_trajectory):
        """
        Compute the collision points of the trajectory with the collision band
        returns (passBehindCollisionPoint, passFrontCollisionPoint)
        :return: returns a tuple with two values, if there is an intersection the value will be a Point2D, if not it will be None
        """
        
        intersection_1, intersection_2 = intersection(obstacle_trajectory, trajectory)
        
        # intersection_1 = trajectory.intersection(obstacle_trajectory[0])
        # intersection_2 = trajectory.intersection(obstacle_trajectory[1])

        return self._select_right_collision_point(intersection_1, trajectory.radius, self.robot.get_location()), self._select_right_collision_point(intersection_2, trajectory.radius, self.robot.get_location())
        


    def _collision_velocity_times_aux(self, collision_point, obs_col, trajectory_radius, v_obstacle, angle = None):
        x_col = float(collision_point[0])
        y_col = float(collision_point[1])

        x_obs_col = obs_col[0]
        y_obs_col = obs_col[1]

        # print("x_col, y_col")
        # print(x_col, y_col)
        # print("x_obs_col, y_obs_col")
        # print(x_obs_col, y_obs_col)

        distance = math.sqrt((x_col - x_obs_col)**2 + (y_col - y_obs_col)**2)

        # print(distance, angle, v_obstacle)

        t = distance/v_obstacle

        if angle == None:
            angle = np.arctan2(2*x_col*y_col, pow(x_col,2)-pow(y_col,2))

        w = angle/t
        v = trajectory_radius*w

        return (t, w, v)

    def _collision_velocity_times(self, obstacle, collision_points, trajectory):
        """
        APENDIX B
        Devuelve tiempo y velocidades minimas y maximas que debe llevar le robot para no colisionar con el obstaculo
        Se llama a esta funcion con cada una de las trayectorias del robot y con cada uno de los obstaculos
        Si no hay puntos de colision no hay limite de velocidad
        :param obstacle: obstaculo con el que se va a comprobar la colision
        :param collision_points: puntos de colision de la trayectoria del robot con el obstaculo, hay un maximo de dos colisiones
        :param trajectory: trayectoria circular del robot
        :return ((t1, w_max, v_max),(t2, w_min, v_min)). velocity_time_space
        """
        # print("trajectory.radius")
        # print(trajectory.radius)
        # if trajectory.radius < 0:
        #     obs_col_behind, obs_col_ahead = obstacle.get_colision_points_2()
        # else:
        #     obs_col_behind, obs_col_ahead = obstacle.get_colision_points_1()
        # obs_col_behind, obs_col_ahead = obstacle.get_colision_points_1()
        # print("obs_col_behind, obs_col_ahead")
        # print(obs_col_behind, obs_col_ahead)
        obs_col_behind, obs_col_ahead = obstacle.get_colision_points()
        # print(obs_col_behind, obs_col_ahead)
        v_object, _ = obstacle.get_speed()

        w_min = v_min = w_max = v_max = t_min = t_max = 0


        if collision_points[0] == None or collision_points[1] == None:
            if collision_points[0] == None and collision_points[1] == None:
                #TODO: Mirar si esto esta bien
                v_min = self.robot.max_v
                w_min = v_min/trajectory.radius

                v_max = self.robot.max_v
                w_max = v_min/trajectory.radius
            else:
                if collision_points[0] == None:
                    collision_point =  collision_points[1]
                else:
                    collision_point =  collision_points[0]
                
                # Calculo la velocidad maxima que puedo llevar
                # Pongo como minima el limite superior. No hay velocidad de escape
                (t_max, w_max, v_max) = self._collision_velocity_times_aux(collision_point, obs_col_behind, trajectory.radius, v_object)
                
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

            if abs(angle_1) > abs(angle_2):
                idx_first = idx_first + 1
            # print(idx_first)
            
            # print("collision_points")
            # print(float(collision_points[idx_first][0]), float(collision_points[idx_first][1]))
            # print("obs_col_behind,obs_col_ahead")
            # print(obs_col_behind, obs_col_ahead)
            (t_max, w_max, v_max) = self._collision_velocity_times_aux(collision_points[idx_first], obs_col_behind, trajectory.radius, v_object, angles[idx_first])

            (t_min, w_min, v_min) = self._collision_velocity_times_aux(collision_points[(idx_first+1)%2], obs_col_ahead, trajectory.radius, v_object, angles[(idx_first+1)%2])



        return [(t_max, w_max, v_max), (t_min, w_min, v_min)]


       
    def _create_DOVS(self, velocity_time_space):
        """
        Dada la lista de velocidades crea el dovs de manera que se le puedan añadir mas dovs
        """

        passBehind, passFront = [(passBehind[0][1], passBehind[0][2]) for passBehind in velocity_time_space], [(passFront[1][1], passFront[1][2]) for passFront in velocity_time_space]


        # #is_inside = polygon.contains_point(point)

        vertices = passBehind + passFront[::-1]
        return vertices


    def _combine_DOVS(self, list_dovs):
        combine_dovs = []
        for dovs in list_dovs:
            combine_dovs = combine_dovs + dovs
        return combine_dovs
        
    def _choose_speed(self):
        v = 1
        w = 1

        if self.robot.velocity_window.contains([v,w]):
            return v,w
        else:
            return 0,0