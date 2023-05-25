# import sympy
# from sympy.geometry import *
import math
from matplotlib import pyplot as plt
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches

import numpy as np
from DOVS.geometry.collision_point import CollisionPoint
from DOVS.geometry.dov import DOV

# from sympy import nsolve, Symbol, symbols
# from sympy import Polygon, plot
from DOVS.geometry.velocity_window import VelocityWindow


from DOVS.object_dovs import DynamicObstacleDOVS, ObjectDOVS, RobotDOVS
from DOVS.plot_dovs import PlotDOVS

from DOVS.geometry.trajectory import intersection

"""
https://matplotlib.org/stable/api/_as_gen/matplotlib.patches.Polygon.html
Buscar funcion que dado un poligono ver si velocidades pertence a ese poligono
https://www.matecdev.com/posts/shapely-polygon-from-points.html
"""


class DOVS:
    def __init__(self, robot, obstacles, timestep, fig_dovs, ax_dovs) -> None:
        self.max_distance = 10

        velocity_window = VelocityWindow(robot, timestep)

        self.robot = RobotDOVS(robot, velocity_window)
        self.obstacles = list(map(lambda obstacle: DynamicObstacleDOVS(obstacle, robot.radius, (robot.x, robot.y, robot.theta), self.max_distance), obstacles))
        self.timestep = timestep

        self.fig_dovs = fig_dovs
        self.ax_dovs = ax_dovs


    def compute_DOVS(self):
        """
        Compute the DOVS algorithm

        This function should be call every timestep
        """
        
        collision_points_list = []
        final_dovs = DOV()
        for obstacle in self.obstacles:
        
            velocity_time_space = []
            #self.robot.trajectories = self.robot.trajectories[1:]
            for robot_trajectory in self.robot.trajectories:
                
                # Compute collision points of an obstacle for every robot trajectory
                collision_points = self._compute_collision_points(robot_trajectory, obstacle.trajectory, obstacle.get_location())
                collision_points_list.append(collision_points)
                
                velocity_time = self._collision_velocity_times(obstacle, collision_points, robot_trajectory)
                if len(velocity_time) != 0: velocity_time_space.append(velocity_time)
                
            dovs = DOV(velocity_time_space)
            final_dovs.combine_DOVS(dovs)
            
        

        plotDOVS = PlotDOVS(self.robot, self.obstacles, self.fig_dovs, self.ax_dovs)#Quiza pasarle el dovs??
        plotDOVS.plot_trajectories(collision_points_list)
        plotDOVS.plot_DOVS(final_dovs)
        

        return self._choose_speed()
        
    
    def _select_right_collision_point(self, collision_points, trajectory_radius, obstacle_position):
        """
        Select the right collision point of the intersection of the robot trajectory and the obstacle trajectory
        obs_col_behind, obs_col_ahead = obstacle.get_colision_points() , cambiar esto collision1_from_obstacle[0] >= obs_col_behind
        Cojer de como mucho dos puntos de colision que estan por delante del obstaculo el que esta mas cerca del robot
        """
        if collision_points:
            collision_point_1 = CollisionPoint(float(collision_points[0][0]), float(collision_points[0][1]), trajectory_radius)

            collision1_from_obstacle = ObjectDOVS.loc(np.dot(np.linalg.inv(ObjectDOVS.hom(obstacle_position)),ObjectDOVS.hom((collision_point_1.x, collision_point_1.y, 0))))
            
            if len(collision_points) == 1:
                if collision1_from_obstacle[0] >= 0:
                    return collision_point_1
                else:
                    return None
            else:
                collision_point_2 = CollisionPoint(float(collision_points[1][0]), float(collision_points[1][1]), trajectory_radius)
                #collision1_from_obstacle = ObjectDOVS.loc(np.dot(np.linalg.inv(ObjectDOVS.hom(obstacle_position)),ObjectDOVS.hom((collision_points[0][0], collision_points[0][1], 0))))
                collision2_from_obstacle = ObjectDOVS.loc(np.dot(np.linalg.inv(ObjectDOVS.hom(obstacle_position)),ObjectDOVS.hom((collision_point_2.x, collision_point_2.y, 0))))
                
                # Devolver el punto de colision mas cercano a la trayectoria del robot

                if collision1_from_obstacle[0] >= 0 and collision2_from_obstacle[0] >= 0:
                    if collision_point_1.angle_arc <= collision_point_2.angle_arc:
                        return collision_point_1
                    else:
                        # print()
                        # print(float(collision_points[1][0]), float(collision_points[1][1]))
                        # print("th: " + str(theta))
                        # print("Arc len: " + str(arclength_2))
                        # print("Arc len2: " + str(collision_point_2.arclength))
                        # print("Angle arc: " + str(angle_arc_2))
                        return collision_point_2
                elif collision1_from_obstacle[0] >= 0:
                    # print()
                    # print(float(collision_points[0][0]), float(collision_points[0][1]))
                    # print("th: " + str(theta))
                    # print("Arc len: " + str(arclength_1))
                    # print("Arc len2: " + str(collision_point_1.arclength))
                    # print("Angle arc: " + str(angle_arc_1))
                    return collision_point_1
                elif collision2_from_obstacle[0] >= 0:
                    # print()
                    # print(float(collision_points[1][0]), float(collision_points[1][1]))
                    # print("th: " + str(theta))
                    # print("Arc len: " + str(arclength_2))
                    # print("Arc len2: " + str(collision_point_2.arclength))
                    # print("Angle arc: " + str(angle_arc_2))
                    return collision_point_2
                else:
                    return None
        else:
            return None
        
    def _valid_collision_points(self, collision_point_1, collision_point_2):
        if collision_point_1 != None and collision_point_2 != None:
            if collision_point_1.angle_arc <= 90 or collision_point_2.angle_arc <= 90:
                return collision_point_1, collision_point_2
            else:
                return None, None
        elif collision_point_1 != None and collision_point_1.angle_arc <= 90:
            return collision_point_1, None
        elif collision_point_2 != None and collision_point_2.angle_arc <= 90:
            return None, collision_point_2
        else:
            return None, None


    def _compute_collision_points(self, trajectory, obstacle_trajectory, obstacle_position):
        """
        Compute the collision points of the trajectory with the collision band
        returns (passBehindCollisionPoint, passFrontCollisionPoint)
        :return: returns a tuple with two values, if there is an intersection the value will be a Point2D, if not it will be None
        """
        
        intersection_1, intersection_2 = intersection(obstacle_trajectory, trajectory)
        
        # intersection_1 = trajectory.intersection(obstacle_trajectory[0])
        # intersection_2 = trajectory.intersection(obstacle_trajectory[1])

        return self._valid_collision_points(self._select_right_collision_point(intersection_1, trajectory.radius, obstacle_position), self._select_right_collision_point(intersection_2, trajectory.radius, obstacle_position))
        


    def _collision_velocity_times_aux(self, collision_point, obs_col, trajectory_radius, v_obstacle, obs_trajectory):
        x_col = collision_point.x
        y_col = collision_point.y

        x_obs_col = obs_col[0]
        y_obs_col = obs_col[1]

        # print("x_col, y_col")
        # print(x_col, y_col)
        # print("x_obs_col, y_obs_col")
        # print(x_obs_col, y_obs_col)
        
        distance = obs_trajectory.distance_between_points(x_col, y_col, x_obs_col, y_obs_col)

        # print(distance, angle, v_obstacle)

        t = distance/v_obstacle

        # if angle == None:
        #     angle = np.arctan2(2*x_col*y_col, pow(x_col,2)-pow(y_col,2))

        w = collision_point.angle/t
        v = trajectory_radius*w

        print()
        print("x_col, y_col: " + str(x_col) + "," + str(y_col))
        #print("Distancia:" + str(distance))
        print("t:" + str(t))
        # print(w, v)

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

        if collision_points[0] == None and collision_points[1] == None: 
            return []
        elif collision_points[0] == None or collision_points[1] == None:
            if collision_points[0] == None:
                collision_point =  collision_points[1]
            else:
                collision_point =  collision_points[0]
            
            # Calculo la velocidad maxima que puedo llevar
            # Pongo como minima el limite superior. No hay velocidad de escape
            (t_max, w_max, v_max) = self._collision_velocity_times_aux(collision_point, obs_col_behind, trajectory.radius, v_object, obstacle.trajectory)
            # if t_max > 7.5:
            #     return []
            w_max, v_max = self.robot.normalize_speed(w_max, v_max, trajectory.radius)

            
            v_min = self.robot.max_v
            w_min = v_min/trajectory.radius
        else:
            idx_first = 0
            
            # Tenemos que ver cual de los dos puntos de colision es el que va a definir la velocidad maxima 
            # x_col_1 = collision_points[0].x
            # y_col_1 = collision_points[0].y

            # x_col_2 = collision_points[1].x
            # y_col_2 = collision_points[1].y


            # angle_1 = np.arctan2(2*x_col_1*y_col_1, pow(x_col_1,2)-pow(y_col_1,2))
            # angle_2 = np.arctan2(2*x_col_2*y_col_2, pow(x_col_2,2)-pow(y_col_2,2))

            # arclength_1 = trajectory.radius * angle_1
            # arclength_2 = trajectory.radius * angle_2

            # print("arclength_1")
            # print(arclength_1)
            # print("arclength_2")
            # print(arclength_2)
            
            #angle = np.arctan2(2*x*y, pow(x,2)-pow(y,2))
            # angles = [angle_1, angle_2]

            if collision_points[0].arclength > collision_points[1].arclength:
                idx_first = idx_first + 1
            # print(idx_first)
            
            # print("collision_points")
            # print(float(collision_points[idx_first][0]), float(collision_points[idx_first][1]))
            # print("obs_col_behind,obs_col_ahead")
            # print(obs_col_behind, obs_col_ahead)
            (t_max, w_max, v_max) = self._collision_velocity_times_aux(collision_points[idx_first], obs_col_behind, trajectory.radius, v_object, obstacle.trajectory)

            (t_min, w_min, v_min) = self._collision_velocity_times_aux(collision_points[(idx_first+1)%2], obs_col_ahead, trajectory.radius, v_object, obstacle.trajectory)

            # if t_max > 7.5 or t_min > 7.5:
            #     return []
            
            w_max, v_max = self.robot.normalize_speed(w_max, v_max, trajectory.radius)
            w_min, v_min = self.robot.normalize_speed(w_min, v_min, trajectory.radius)

        return [(t_max, w_max, v_max), (t_min, w_min, v_min)]




        
    def _choose_speed(self):
        return 0.0, 0.0