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
        trajectories = self.compute_trajectories()

        for obstacle in self.obstacles:
            # Compute the trajectory of the obstacle
            collision_band = self.compute_collision_band()
            

            velocity_time_space = []
            for trajectory in trajectories:
                collision_points = self.compute_collision_points(trajectory, collision_band)

                velocity_time = self.collision_velocity_times(collision_points)
                velocity_time_space.append(velocity_time)

            dovs = self.create_DOVS()#Calculamos el dovs para ese objeto
            self.append_DOVS(list_dovs, dovs)#Añadimos el dovs calculado al dovs total, geometrically merged

        
        self.plot_DOVS(velocity_time_space)
        



    def compute_collision_points(self, trajectory, collision_band):
        """
        Compute the collision points of the trajectory with the collision band
        returns (passBehindCollisionPoint, passFrontCollisionPoint)
        """
        pass

    def collision_velocity_times(self, obstacle, collision_points):
        """
        APENDIX B
        Devuelve tiempo y velocidades minimas y maximas que debe llevar le robot para no colisionar
        :return ((t1, v_max, w_max),(t2, v_min, w_min))
        """
        pass

        
    def plot_DOVS(self, velocity_time_space):
        # Dibujar las velocidades de velocity_time_space
        pass



class ObjectDOVS():
    def __init__(self, v, w, x, y, theta) -> None:
        self.v = v
        self.w = w
        self.x = x
        self.y = y
        self.theta = theta    


class DynamicObstacleDOVS(ObjectDOVS):
    def __init__(self, obstacle, robot_radius) -> None:
        # Colision band

        # El tamaño del objeto es el cuadrado que rodea al circulo
        self.size = obstacle.radius + robot_radius

        super().__init__(obstacle.v, obstacle.w, obstacle.x, obstacle.y, obstacle.theta)
    
    def compute_collision_band(self):
        """
        Returns the collision band of the obstacle(two trajectories)
        [passBehind, passFront]
        """
        pass



class RobotDOVS(ObjectDOVS):
    def __init__(self, robot) -> None:
        self.robot = robot

        self.x_goal = robot.x_goal
        self.y_goal = robot.y_goal
        self.min_v = robot.min_v
        self.max_v = robot.max_v
        self.min_w = robot.min_w
        self.max_w = robot.max_w
        self.max_av = robot.max_av
        self.max_aw = robot.max_aw

        self.trajectory_radius_max = 

        super.__init__(robot.v, robot.w, robot.x, robot.y, robot.theta)
    
    def compute_trajectories(self):
        """
        Compute all the possible trajectories of the robot
        discretized set of feasible circular trajectories
        """
        pass

        