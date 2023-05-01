from matplotlib import pyplot as plt
import matplotlib.patches as patches
import numpy as np

from sympy.plotting import plot
from sympy.plotting import plot_implicit
from sympy.geometry import Point, Circle, Line
  


"""Gets and prints the spreadsheet's header columns

:param file_loc: The file location of the spreadsheet
:type file_loc: str
:param print_cols: A flag used to print the columns to the console
    (default is False)
:type print_cols: bool
:returns: a list of strings representing the header columns
:rtype: list
"""

class PlotDOVS:
    """
    A class for plotting the trajectories of the robot and dynamic obstacles.

    Attributes:
        robot (Robot): The robot object.
        obstacles (list): A list of DynamicObstacle objects.

    Methods:
        plot_trajectories(robot_trajectories, obstacles_trajectory, collision_points_list):
            Plots the trajectories of the robot and dynamic obstacles, as well as any collision points.
    """

    def __init__(self, robot, obstacles) -> None:
        self.plot_robot = PlotRobotDOVS(robot)
        self.plot_obstacles = list(map(lambda obstacle: PlotDynamicObstacleDOVS(obstacle), obstacles))

    def prueba(self):
        # using Circle()
        c1 = Circle(Point(0, 0), 5)
        print(c1.equation())
        p1 = Point(1, 2)
        p2 = Point(3, 4)

        # equation of the line using two points
        line = Line(p1, p2)
        #print(c1.hradius, c1.vradius, c1.radius)
        p1 = plot_implicit(line.equation(), show=False)
        p1.show()



    def plot_trajectories(self, robot_trajectories, obstacles_trajectory, collision_points_list):
        """
        Plots the trajectories of the robot and dynamic obstacles, as well as any collision points.

        Args:
            robot_trajectories (list): A list of RobotTrajectory objects representing the robot's trajectory.
            obstacles_trajectory (list): A list of lists of ObstacleTrajectory objects representing the dynamic obstacles' trajectories.
            collision_points_list (list): A list of lists of CollisionPoint objects representing any collision points.
        """
        fig, ax = plt.subplots()

        self.plot_robot.plot_position(ax)

        self.plot_robot.plot_trajectories(ax, robot_trajectories)

        for i, obstacle in enumerate(self.plot_obstacles):
            obstacle.plot_position(ax)
            obstacle.plot_colision_points(ax)
            obstacle.plot_trajectories(ax, obstacles_trajectory[i])
        
        for collision_points in collision_points_list:
            for collision_point in collision_points:
                if collision_point != None:
                    ax.plot(collision_point.coordinates[0], collision_point.coordinates[1], 'g.')
                    

        # Set the axis limits
        plt.axis('equal')
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)

        plt.show()

    def plot_DOVS(self, dovs, velocity_window):
        polygon = patches.Polygon(dovs, closed=True, facecolor='green')
        # polygon = patches.Polygon(vertices, facecolor='red', edgecolor='black')

        # Crear la figura y los ejes para mostrar los polígonos
        fig, ax = plt.subplots()

        ax.add_patch(polygon)


        # Dibujar rombo de velocidades#TODO
        self.plot_robot.plot_velocity_window(ax)

        ax.set_xlim([-1, 2])
        ax.set_ylim([-1, 2])

        # Mostrar la figura
        plt.show()

    
class PlotObjectDOVS:
    """
    A base class for plotting objects in the simulation.

    Methods:
        dibrobot(pos, axis, c):
            Draws the robot at a given position on the plot.
    """

    def dibrobot(self, pos, axis, c):
        """
        Draws the robot at a given position on the plot.

        Args:
            pos (tuple): A tuple representing the x, y, and theta coordinates of the robot.
            axis (AxesSubplot): The subplot to draw the robot on.
            c (str): The color to draw the robot in.
        """
        largo=0.1
        corto=0.05
        descentre=0.01

        trasera_dcha=np.array([-largo,-corto,1])
        trasera_izda=np.array([-largo,corto,1])
        delantera_dcha=np.array([largo,-corto,1])
        delantera_izda=np.array([largo,corto,1])
        frontal_robot=np.array([largo,0,1])
        tita=pos[2]
        Hwe=np.array([[np.cos(tita), -np.sin(tita), pos[0]],
                    [np.sin(tita), np.cos(tita), pos[1]],
                    [0,        0 ,        1]])
        Hec=np.array([[1,0,descentre],
                    [0,1,0],
                    [0,0,1]])
        extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
        robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
        axis.plot(robot[0,:], robot[1,:], c)



class PlotDynamicObstacleDOVS(PlotObjectDOVS):
    """
    Clase que representa la visualización de un obstáculo dinámico en el simulador DOVS.

    ...

    Atributos
    ----------
    obstacle : DynamicObstacle
        Objeto de la clase DynamicObstacle que representa el obstáculo dinámico a visualizar.

    Métodos
    -------
    plot_trajectories(axis, trajectories)
        Dibuja las trayectorias del obstáculo dinámico en el eje especificado.
    plot_position(axis)
        Dibuja la posición actual del obstáculo dinámico en el eje especificado.
    """

    def __init__(self, obstacle) -> None:
        self.obstacle = obstacle

    # TODO: Maybe only plot as going straight and not backwords too
    def plot_trajectories(self, axis, trajectories):
        """
        Dibuja las trayectorias del obstáculo dinámico en el eje especificado.

        Parámetros
        ----------
        axis : matplotlib.axes._subplots.AxesSubplot
            Eje en el que se dibujarán las trayectorias.
        trajectories : list[DOVSTrajectory]
            Lista de objetos DOVSTrajectory que representan las trayectorias del obstáculo dinámico a dibujar.
        """

        # print("Plotting obstacle trajectories")
        # print(trajectories)
        
        #TODO: Hago clases distintas para los tipos de trayectorias???
        for trajectory in trajectories:
            # p1 = plot_implicit(trajectory.equation(), show=False)
            if self.obstacle.w != 0:
                axis.add_patch(plt.Circle((trajectory.center.coordinates[0], trajectory.center.coordinates[1]), trajectory.radius, color='green', fill=False))
            else:
            # # print(trajectory)
                point1 = trajectory.points[0].coordinates
                point2 = trajectory.points[1].coordinates

                axis.axline(xy1=tuple(float(coord) for coord in point1), xy2=tuple(float(coord) for coord in point2))
        

    def plot_position(self, axis):
        """
        Dibuja la posición actual del obstáculo dinámico en el eje especificado.

        Parámetros
        ----------
        axis : matplotlib.axes._subplots.AxesSubplot
            Eje en el que se dibujará la posición actual del obstáculo dinámico.
        """
        self.dibrobot(self.obstacle.get_location(), axis, 'b')
        axis.add_patch(plt.Circle((self.obstacle.x, self.obstacle.y), self.obstacle.radius, color='blue', fill=False))


    def plot_colision_points(self, axis):
        col_behind, col_ahead = self.obstacle.get_colision_points()
        axis.plot(col_behind[0], col_behind[1], 'k.')
        axis.plot(col_ahead[0], col_ahead[1], 'k.')

class PlotRobotDOVS(PlotObjectDOVS):
    """
    Clase que representa la visualización de un robot en el simulador DOVS.

    ...

    Atributos
    ----------
    robot : Robot
        Objeto de la clase Robot que representa el robot a visualizar.

    Métodos
    -------
    plot_trajectories(axis, trajectories)
        Dibuja las trayectorias del robot en el eje especificado.
    plot_position(axis)
        Dibuja la posición actual del robot en el eje especificado.
    """

    def __init__(self, robot) -> None:
        self.robot = robot

    def plot_trajectories(self, axis, trajectories):
        for trajectory in trajectories:
            axis.add_patch(plt.Circle((trajectory.center.coordinates[0], trajectory.center.coordinates[1]), trajectory.radius, color='blue', fill=False))

    def plot_position(self, axis):
        #axis.scatter(self.robot.x, self.robot.y, color='red', marker='^', s=100, angle=0)

        #axis.plot(self.robot.x, self.robot.y, 'r.')
        self.dibrobot(self.robot.get_location(), axis, 'r')
        #axis.plot(self.robot.x, self.robot.y, marker=(3, 0, self.robot.theta*90), markersize=20, linestyle='None')

    def plot_velocity_window(self, axis):
        axis.plot(self.robot.v, self.robot.w, 'k.')
        # TODO cambia de donde sale esto
        t = 0.2 #timestep
        #a = (vf - vi) / t
        v_max = self.robot.max_av * t + self.robot.v
        v_min = -self.robot.max_av * t + self.robot.v
        w_max = self.robot.max_aw * t + self.robot.v
        w_min = -self.robot.max_aw * t + self.robot.v
        # axis.plot(v_max, self.robot.w, 'k.')
        # axis.plot(v_min, self.robot.w, 'k.')
        # axis.plot(self.robot.v, w_max, 'k.')
        # axis.plot(self.robot.v, w_min, 'k.')

        poly = patches.Polygon(list(zip([v_max, self.robot.v, v_min, self.robot.v], [self.robot.w, w_max, self.robot.w, w_min])), color='lightblue', alpha=0.5)

        # Agrega el polígono al eje
        axis.add_patch(poly)
        # v = self.robot.max_av * t


