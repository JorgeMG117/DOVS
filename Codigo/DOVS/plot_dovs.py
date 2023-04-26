from matplotlib import pyplot as plt
import numpy as np

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
        """
        Initializes a new instance of the PlotDOVS class.

        Args:
            robot (Robot): The robot object.
            obstacles (list): A list of DynamicObstacle objects.
        """
        self.plot_robot = PlotRobotDOVS(robot)
        self.plot_obstacles = list(map(lambda obstacle: PlotDynamicObstacleDOVS(obstacle), obstacles))

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

    def plot_DOVS(self, velocity_time_space):
        # # fig, ax = plt.subplots()
        # # Dibujar las velocidades de velocity_time_space
        # print(velocity_time_space)

        # # Extract x and y coordinates from points
        # v = [point[1] for point in velocity_time_space]
        # w = [point[2] for point in velocity_time_space]

        # points = [(point[1], point[2]) for point in velocity_time_space]

        # # Add first point to end of lists to close the polygon
        # # v.append(v[0])
        # # w.append(w[0])
        # polygon = plt.Polygon(points, closed=True, fill=None)


        # fig, ax = plt.subplots()
        # ax.add_patch(polygon)
        # # ax.plot(v, w, 'bo')
        # plt.xlabel('v')
        # plt.ylabel('w')
        # plt.axis('equal')
        # plt.show()

        pass

    
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
        """
        Inicializa un objeto de la clase PlotDynamicObstacleDOVS.

        Parámetros
        ----------
        obstacle : DynamicObstacle
            Objeto de la clase DynamicObstacle que representa el obstáculo dinámico a visualizar.
        """
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
        
        for trajectory in trajectories:
            # print(trajectory)
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
        """
        Inicializa un objeto de la clase PlotRobotDOVS.

        Parámetros
        ----------
        robot : Robot
            Objeto de la clase Robot que representa el robot a visualizar.
        """
        self.robot = robot

    def plot_trajectories(self, axis, trajectories):
        for trajectory in trajectories:
            axis.add_patch(plt.Circle((trajectory.center.coordinates[0], trajectory.center.coordinates[1]), trajectory.radius, color='blue', fill=False))

    def plot_position(self, axis):
        #axis.scatter(self.robot.x, self.robot.y, color='red', marker='^', s=100, angle=0)

        #axis.plot(self.robot.x, self.robot.y, 'r.')
        self.dibrobot(self.robot.get_location(), axis, 'r')
        #axis.plot(self.robot.x, self.robot.y, marker=(3, 0, self.robot.theta*90), markersize=20, linestyle='None')



