from matplotlib import pyplot as plt
import matplotlib.patches as patches
import numpy as np

# from sympy.plotting import plot
# from sympy.plotting import plot_implicit
# from sympy.geometry import Point, Circle, Line


class PlotDOVS:

    def __init__(self, robot, obstacles) -> None:
        self.plot_robot = PlotRobotDOVS(robot)
        self.plot_obstacles = list(map(lambda obstacle: PlotDynamicObstacleDOVS(obstacle), obstacles))

    # def prueba(self):
    #     # using Circle()
    #     c1 = Circle(Point(0, 0), 5)
    #     print(c1.equation())
    #     p1 = Point(1, 2)
    #     p2 = Point(3, 4)

    #     # equation of the line using two points
    #     line = Line(p1, p2)
    #     #print(c1.hradius, c1.vradius, c1.radius)
    #     p1 = plot_implicit(line.equation(), show=False)
    #     p1.show()



    def plot_trajectories(self, robot_trajectories, obstacles_trajectory, collision_points_list):
        fig, ax = plt.subplots()

        self.plot_robot.plot_position(ax)

        self.plot_robot.plot_trajectories(ax, robot_trajectories)

        for i, obstacle in enumerate(self.plot_obstacles):
            obstacle.plot_position(ax)
            #print(obstacle.obstacle.radius)
            obstacle.plot_colision_points(ax)
            obstacle.plot_trajectory(ax, obstacles_trajectory[i])
        
        for collision_points in collision_points_list:
            i = 0
            for collision_point in collision_points:
                if collision_point != None:
                    if i == 0:
                        ax.plot(collision_point[0], collision_point[1], 'k.')
                    else:
                        ax.plot(collision_point[0], collision_point[1], 'b.')
                i = i + 1
                    

        # Set the axis limits
        plt.axis('equal')
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)

        #plt.show()

    def plot_DOVS(self, dovs):
        print("dovs")
        print(dovs)
        print("fin")
        
        

        passBehind, passFront = [(passBehind[0][1], passBehind[0][2]) for passBehind in dovs], [(passFront[1][1], passFront[1][2]) for passFront in dovs]
        
        # print(len(passBehind))
        # print(len(passFront))

        # #is_inside = polygon.contains_point(point)

        #vertices = passBehind + passFront[::-1]
        

        
        mitad_1 = passFront[:int(np.round((len(passFront)+1)/2))]
        mitad_1 = mitad_1[::-1]#Dar la vuelta
        print(len(mitad_1))

        mitad_2 = passFront[int(np.round((len(passFront)+1)/2)):]
        mitad_2 = mitad_2[::-1]
        print(len(mitad_2))

        vertices = mitad_1 + mitad_2


        mitad_1 = passBehind[:int(np.round((len(passBehind)+1)/2))]
        #mitad_1 = mitad_1[::-1]#Dar la vuelta
        
        mitad_2 = passBehind[int(np.round((len(passBehind)+1)/2)):]
        # mitad_2 = mitad_2[::-1]
        
        vertices = vertices + mitad_2 + mitad_1
    

        polygon = patches.Polygon(vertices, closed=False, facecolor='green')
        # polygon_2 = patches.Polygon(vertices_2, closed=False, facecolor='purple')
        # polygon = patches.Polygon(vertices, facecolor='red', edgecolor='black')

        # plt.ion()
        # Crear la figura y los ejes para mostrar los polígonos
        fig, ax = plt.subplots()


        ax.add_patch(polygon)
       

        for point in passBehind:
            # print(point)
            # input("Press enter to continue...")
            ax.plot(point[0], point[1], "k.")
            # fig.canvas.draw()   # Redraw the plot
            # plt.pause(0.5)
            
        for point in passFront:
            # print(point)
            
            # input("Press enter to continue...")
            ax.plot(point[0], point[1], "b.")
            # fig.canvas.draw()   # Redraw the plot
            # plt.pause(0.5)

        ax.plot(mitad_1[0][0], mitad_1[0][1], "r*")
        ax.plot(mitad_1[len(mitad_1)-1][0], mitad_1[len(mitad_1)-1][1], "y*")
        ax.plot(mitad_2[0][0], mitad_2[0][1], "r*")
        ax.plot(mitad_2[len(mitad_2)-1][0], mitad_2[len(mitad_2)-1][1], "y*")
        # ax.plot(passBehind[0][0], passBehind[0][1], "r*")
        # ax.plot(passBehind[len(passBehind)-1][0], passBehind[len(passBehind)-1][1], "y*")
        # ax.plot(passFront[5][0], passFront[5][1], "r*")
        # ax.plot(passFront[len(passFront)-1][0], passFront[len(passFront)-1][1], "y*")

        # ax.plot(passBehind[1][0], passBehind[1][1], "y*")
        # ax.plot(passBehind[len(passBehind)-2][0], passBehind[len(passBehind)-1][1], "y*")
        # ax.plot(passFront[1][0], passFront[1][1], "y*")
        # ax.plot(passFront[len(passFront)-2][0], passFront[len(passFront)-2][1], "y*")
          

        # Dibujar rombo de velocidades#TODO
        self.plot_robot.plot_velocity_window(ax)

        plt.axis('equal')
        ax.set_xlim([-0.5, 2])
        ax.set_ylim([-0.5, 2])
        # ax.set_xlim([self.plot_robot.robot.min_w, self.plot_robot.robot.max_w])
        # ax.set_ylim([self.plot_robot.robot.min_v, self.plot_robot.robot.max_v])

        # Mostrar la figura
        plt.show()

    
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
    def plot_trajectory(self, axis, trajectory):
        trajectory.plot(axis)

        # print("Plotting obstacle trajectories")
        # print(trajectories)
        
        #TODO: Hago clases distintas para los tipos de trayectorias???
        # for trajectory in trajectories:
        #     trajectory.plot(axis)
            # p1 = plot_implicit(trajectory.equation(), show=False)

            # if self.obstacle.w != 0:
            #     axis.add_patch(plt.Circle((trajectory.center.coordinates[0], trajectory.center.coordinates[1]), trajectory.radius, color='green', fill=False))
            # else:
            # # # print(trajectory)
            #     point1 = trajectory.points[0].coordinates
            #     point2 = trajectory.points[1].coordinates

            #     axis.axline(xy1=tuple(float(coord) for coord in point1), xy2=tuple(float(coord) for coord in point2))
        

    def plot_position(self, axis):
        self.dibrobot(self.obstacle.get_location(), axis, 'b')
        axis.add_patch(plt.Circle((self.obstacle.x, self.obstacle.y), self.obstacle.radius, color='blue', fill=False))


    def plot_colision_points(self, axis):
        
        col_behind, col_ahead = self.obstacle.get_colision_points()
        axis.plot(col_behind[0], col_behind[1], 'k.')
        axis.plot(col_ahead[0], col_ahead[1], 'b.')


class PlotRobotDOVS(PlotObjectDOVS):

    def __init__(self, robot) -> None:
        self.robot = robot

    def plot_trajectories(self, axis, trajectories):
        for trajectory in trajectories:
            trajectory.plot(axis)

    def plot_position(self, axis):
        #axis.scatter(self.robot.x, self.robot.y, color='red', marker='^', s=100, angle=0)

        #axis.plot(self.robot.x, self.robot.y, 'r.')
        self.dibrobot(self.robot.get_location(), axis, 'r')
        #axis.plot(self.robot.x, self.robot.y, marker=(3, 0, self.robot.theta*90), markersize=20, linestyle='None')

    def plot_velocity_window(self, axis):
        self.robot.velocity_window.plot(axis)


