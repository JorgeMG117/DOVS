from sympy.geometry import *
import math
from matplotlib import pyplot as plt

def intersection(obstacle_trajectory, robot_trajectory):
    intersection_1 = robot_trajectory.c.intersection(obstacle_trajectory.l1)
    intersection_2 = robot_trajectory.c.intersection(obstacle_trajectory.l2)
    # Transformar lista de puntos a coordenads
    return intersection_1, intersection_2


class ObstacleTrajectory():
    def __init__(self, l1, l2) -> None:
        self.l1 = l1
        self.l2 = l2
    
    def plot(self, axis):
        pass

class LinearTrajectory(ObstacleTrajectory):
    def __init__(self, point1, point2, angle) -> None:
        ray_1 = Ray((0,0), angle=angle)
        l1 = Line(point1, slope=ray_1.slope)
        l2 = Line(point2, slope=ray_1.slope)

        super().__init__(l1, l2)

    def plot(self, axis):
        point1 = self.l1.points[0].coordinates
        point2 = self.l1.points[1].coordinates

        axis.axline(xy1=tuple(float(coord) for coord in point1), xy2=tuple(float(coord) for coord in point2))

        point1 = self.l2.points[0].coordinates
        point2 = self.l2.points[1].coordinates
        axis.axline(xy1=tuple(float(coord) for coord in point1), xy2=tuple(float(coord) for coord in point2))

    
    
class CircularTrajectory(ObstacleTrajectory):
    def __init__(self, point1, point2, position, velocity) -> None:
        x1, y1 = point1
        x2, y2 = point2
        v, w = velocity
        x, y, th = position

        radius = v/w

        center_x = x + radius * math.cos(th + math.pi/2)
        center_y = y + radius * math.sin(th + math.pi/2)

        radius_1 = ((center_x - x1) ** 2 + (center_y - y1) ** 2) ** 0.5
        radius_2 = ((center_x - x2) ** 2 + (center_y - y2) ** 2) ** 0.5  

        l1 = Circle(Point(center_x, center_y), radius_1)
        l2 = Circle(Point(center_x, center_y), radius_2)

        super().__init__(l1, l2)
    
    def plot(self, axis):
        axis.add_patch(plt.Circle((self.l1.center.coordinates[0], self.l1.center.coordinates[1]), self.l1.radius, color='green', fill=False))
    
        axis.add_patch(plt.Circle((self.l2.center.coordinates[0], self.l2.center.coordinates[1]), self.l2.radius, color='green', fill=False))    

    
class RobotTrajectory():
    def __init__(self, radius, position) -> None:
        self.radius = radius
        x, y, th = position
        center_x = x + radius * math.cos(th + math.pi/2)
        center_y = y + radius * math.sin(th + math.pi/2)
        self.c = Circle(Point(center_x, center_y), radius)

    def plot(self, axis):
        axis.add_patch(plt.Circle((self.c.center.coordinates[0], self.c.center.coordinates[1]), self.c.radius, color='blue', fill=False))
