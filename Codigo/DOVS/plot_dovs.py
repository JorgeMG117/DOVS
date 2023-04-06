from matplotlib import pyplot as plt


class PlotDOVS:
    def __init__(self, robot, obstacles) -> None:
        self.plot_robot = PlotRobotDOVS(robot)
        self.plot_obstacles = list(map(lambda obstacle: PlotDynamicObstacleDOVS(obstacle), obstacles))

    def plot_trajectories(self, robot_trajectories, obstacles_trajectory, collision_points_list):
        # Create a figure and axis objects
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
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)

        plt.show()

    def plot_DOVS(self, velocity_time_space):
        # Dibujar las velocidades de velocity_time_space
        pass



class PlotDynamicObstacleDOVS:

    def __init__(self, obstacle) -> None:
        self.obstacle = obstacle

    # TODO: Maybe only plot as going straight and not backwords too
    def plot_trajectories(self, axis, trajectories):
        # print("Plotting obstacle trajectories")
        # print(trajectories)
        for trajectory in trajectories:
            # print(trajectory)
            point1 = trajectory.points[0].coordinates
            point2 = trajectory.points[1].coordinates

            axis.axline(xy1=tuple(float(coord) for coord in point1), xy2=tuple(float(coord) for coord in point2))
    

    def plot_position(self, axis):
        axis.add_patch(plt.Circle((self.obstacle.x, self.obstacle.y), self.obstacle.radius, color='blue', fill=False))



class PlotRobotDOVS:

    def __init__(self, robot) -> None:
        self.robot = robot

    def plot_trajectories(self, axis, trajectories):
        for trajectory in trajectories:
            axis.add_patch(plt.Circle((trajectory.center.coordinates[0], trajectory.center.coordinates[1]), trajectory.radius, color='blue', fill=False))
        

    def plot_position(self, axis):
        axis.plot(self.robot.x, self.robot.y, 'r.')


