import sympy
from sympy.geometry import *
import math
import matplotlib.pyplot as plt

class DOVS:
    def __init__(self, robot, obstacles, timestep) -> None:
        self.robot = RobotDOVS(robot)
        self.obstacles = list(map(lambda obstacle: DynamicObstacleDOVS(obstacle, robot.radius), obstacles))
        self.timestep = timestep

    def compute_DOVS(self):
        """
        Compute the DOVS algorithm

        This function should be call every timestep
        """

        
        # Obtain all pasible trajectories of robot
        robot_trajectories = self.robot.compute_trajectories()
        
        obstacles_trajectory = []
        for obstacle in self.obstacles:
            # Compute the trajectory of the obstacle
            obstacle_trajectory = obstacle.compute_collision_band()

            obstacles_trajectory.append(obstacle_trajectory)
        
            velocity_time_space = []
            collision_points_list = []
            for trajectory in robot_trajectories:
                collision_points = self.compute_collision_points(trajectory, obstacle_trajectory)
                # print("Collision points")
                # print(collision_points)#Tupla de dos elementos cada uno de ellos con dos puntos 2d
                collision_points_list.append(collision_points)

                # velocity_time = self.collision_velocity_times(obstacle, collision_points)
                # velocity_time_space.append(velocity_time)
            
            

            # dovs = self.create_DOVS(velocity_time_space)#Calculamos el dovs para ese objeto

        self.plot_trajectories(robot_trajectories, obstacles_trajectory, collision_points_list)
        #     self.append_DOVS(list_dovs, dovs)#Añadimos el dovs calculado al dovs total, geometrically merged

        
        # self.plot_DOVS(velocity_time_space)
        
        



    def compute_collision_points(self, trajectory, collision_band):
        """
        Compute the collision points of the trajectory with the collision band
        returns (passBehindCollisionPoint, passFrontCollisionPoint)
        """
        intersection_1 = trajectory.intersection(collision_band[0])
        intersection_2 = trajectory.intersection(collision_band[1])

        return intersection_1, intersection_2
    
    def plot_trajectories(self, robot_trajectories, obstacles_trajectory, collision_points_list):
        # Create a figure and axis objects
        fig, ax = plt.subplots()

        self.robot.plot_position(ax)

        self.robot.plot_trajectories(ax, robot_trajectories)

        for i, obstacle in enumerate(self.obstacles):
            obstacle.plot_position(ax)
            obstacle.plot_trajectories(ax, obstacles_trajectory[i])
        
        for collision_points in collision_points_list:
            for collision_point in collision_points:
                if collision_point:
                    ax.plot(collision_point[0].coordinates[0], collision_point[0].coordinates[1], 'g.')
                    ax.plot(collision_point[1].coordinates[0], collision_point[1].coordinates[1], 'g.')
                    

        # Set the axis limits
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)

        plt.show()
        


    def collision_velocity_times(self, obstacle, collision_points):
        """
        APENDIX B
        Devuelve tiempo y velocidades minimas y maximas que debe llevar le robot para no colisionar
        :return ((t1, v_max, w_max),(t2, v_min, w_min))
        """
        pass

    def create_DOVS(velocity_time_space):
        """
        Dada la lista de velocidades crea el dovs de manera que se le puedan añadir mas dovs
        """
        pass

        
    def plot_DOVS(self, velocity_time_space):
        # Dibujar las velocidades de velocity_time_space
        pass




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

    def plot_trajectories(self, axis, trajectories):
        pass

    def plot_position(self, axis):
        pass
        

class DynamicObstacleDOVS(ObjectDOVS):
    def __init__(self, obstacle, robot_radius) -> None:
        # Colision band

        # El tamaño del objeto es el cuadrado que rodea al circulo
        self.radius = obstacle.radius + robot_radius

        super().__init__(obstacle.v, obstacle.w, obstacle.x, obstacle.y, obstacle.theta)
    
    def compute_collision_band(self):
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
    
    def plot_trajectories(self, axis, trajectories):
        # print("Plotting obstacle trajectories")
        # print(trajectories)
        for trajectory in trajectories:
            # print(trajectory)
            x1, y1 = trajectory.points[0].coordinates
            x2, y2 = trajectory.points[1].coordinates
            
            axis.plot([x1,x2], [y1,y2], 'r-')
            # axis.axline((round(x1), round(y1)), (round(x2), round(y2)), linewidth=4, color='r')
            # break
    

    def plot_position(self, axis):
        axis.add_patch(plt.Circle((self.x, self.y), self.radius, color='blue', fill=False))



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
    
    def plot_trajectories(self, axis, trajectories):
        for trajectory in trajectories:
            axis.add_patch(plt.Circle((trajectory.center.coordinates[0], trajectory.center.coordinates[1]), trajectory.radius, color='blue', fill=False))
        

    def plot_position(self, axis):
        axis.plot(self.x, self.y, 'r.')

        