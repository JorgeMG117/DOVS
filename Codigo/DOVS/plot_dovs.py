from matplotlib import pyplot as plt
import numpy as np

"""
fig, ax = plt.subplots(2,2)
fig.set_facecolor('lightgrey')
ax[0,0].plot(data_x, data_y, 'r-')
ax[0,1].plot(data_x, data_y, 'b-')
fig.delaxes(ax[1,0])
ax[1,1].plot(data_x, data_y, 'g-')

"""

class PlotDOVS:

    def __init__(self, robot, obstacles, fig_dovs, ax_dovs) -> None:
        self.plot_robot = PlotRobotDOVS(robot)
        self.plot_obstacles = list(map(lambda obstacle: PlotDynamicObstacleDOVS(obstacle), obstacles))

        self.fig = fig_dovs
        self.ax_trajectories, self.ax_dovs = ax_dovs


    def plot_trajectories(self, collision_points_list):
        self.ax_trajectories.title.set_text("Robocentric view")
        self.ax_trajectories.set_xlabel("x (m)")
        self.ax_trajectories.set_ylabel("y (m)")
        self.plot_robot.plot_position(self.ax_trajectories)

        self.plot_robot.plot_goal(self.ax_trajectories)

        self.plot_robot.plot_trajectories(self.ax_trajectories)

        for obstacle in self.plot_obstacles:
            obstacle.plot_position(self.ax_trajectories)
            #print(obstacle.obstacle.radius)
            obstacle.plot_colision_points(self.ax_trajectories)
            obstacle.plot_trajectory(self.ax_trajectories)
        
        for collision_points in collision_points_list:
            for collision_point in collision_points:
                if collision_point != None:
                    collision_point.plot(self.ax_trajectories)
                    

        self.ax_trajectories.set_xlim(-4, 4)
        self.ax_trajectories.set_ylim(-4, 4)

        #plt.axis('equal')

        # plt.show()

    def plot_DOVS(self, dovs):
        self.ax_dovs.title.set_text("DOVS")
        self.ax_dovs.set_xlabel("w")
        self.ax_dovs.set_ylabel("v")

        # plt.ion()
        # # Crear la figura y los ejes para mostrar los polígonos
        
        
        # self.ax_dovs.clear()

        dovs.plot(self.ax_dovs)

        self.plot_robot.plot_velocity_window(self.ax_dovs)

        self.plot_robot.plot_trajactory_goal(self.ax_dovs)

        #self.ax_dovs.axis('equal')

        self.ax_dovs.set_xlim([self.plot_robot.robot.min_w, self.plot_robot.robot.max_w])
        self.ax_dovs.set_ylim([0, self.plot_robot.robot.max_v])
        # ax.set_xlim([-0.5, 2])
        # ax.set_ylim([-0.5, 2])

        # plt.draw()
        # plt.pause(0.1)
        #plt.axis('equal')
        
        

        # Mostrar la figura
        # plt.show()

    
class PlotObjectDOVS:

    def dibrobot(self, pos, axis, c):
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

    def __init__(self, obstacle) -> None:
        self.obstacle = obstacle

    # TODO: Maybe only plot as going straight and not backwords too
    def plot_trajectory(self, axis):
        self.obstacle.trajectory.plot(axis)        


    def plot_position(self, axis):
        self.dibrobot(self.obstacle.get_location(), axis, 'b')
        axis.add_patch(plt.Circle((self.obstacle.x, self.obstacle.y), self.obstacle.radius, color='blue', fill=False))


    def plot_colision_points(self, axis):
        
        col_behind, col_ahead = self.obstacle.get_colision_points()
        axis.plot(col_behind[0], col_behind[1], 'g.')
        axis.plot(col_ahead[0], col_ahead[1], 'r.')


class PlotRobotDOVS(PlotObjectDOVS):

    def __init__(self, robot) -> None:
        self.robot = robot

    def plot_trajectories(self, axis):
        for trajectory in self.robot.trajectories:
            trajectory.plot(axis)

    def plot_position(self, axis):
        #axis.scatter(self.robot.x, self.robot.y, color='red', marker='^', s=100, angle=0)

        #axis.plot(self.robot.x, self.robot.y, 'r.')
        self.dibrobot(self.robot.get_location(), axis, 'r')
        #axis.plot(self.robot.x, self.robot.y, marker=(3, 0, self.robot.theta*90), markersize=20, linestyle='None')

    def plot_velocity_window(self, axis):
        self.robot.velocity_window.plot(axis)

    def plot_goal(self, axis):
        axis.plot(self.robot.x_goal, self.robot.y_goal, 'r*')

    def plot_trajactory_goal(self, axis):
        w, v = self.robot.get_speed_goal()
        axis.plot([0, w], [0, v], color='gray')
        

