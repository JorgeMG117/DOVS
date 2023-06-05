from sympy.geometry import *
import math
from matplotlib import pyplot as plt

def intersection(obstacle_trajectory, robot_trajectory):
    intersection_1 = robot_trajectory.c.intersection(obstacle_trajectory.l1)
    intersection_2 = robot_trajectory.c.intersection(obstacle_trajectory.l2)
    
    return intersection_1, intersection_2


class ObstacleTrajectory():
    def __init__(self, l1, l2) -> None:
        self.l1 = l1
        self.l2 = l2
    
    def plot(self, axis):
        pass

    def distance_between_points(self, x1, y1, x2, y2):
        pass

class LinearTrajectory(ObstacleTrajectory):
    def __init__(self, point1, point2, angle, max_distance) -> None:
        self.angle = angle
        self.max_distance = max_distance
        
        ray_1 = Ray((0,0), angle=angle)
    
        l1 = Line(point1, slope=ray_1.slope)
        l2 = Line(point2, slope=ray_1.slope)


        super().__init__(l1, l2)#l1 a la derecha del objeto

    def plot(self, axis):
        """
        Plots the linear trajectory on the given axis.

        Args:
            axis: The axis on which to plot the trajectory.
        """
        
        dx = self.max_distance * math.cos(self.angle)
        dy = self.max_distance * math.sin(self.angle)
        l1_x1,l1_y1 = self.l1.points[0]
        l2_x1,l2_y1 = self.l2.points[0]
        
        # Compute the end point of the line using the large distance
        x_end = l1_x1 + dx
        y_end = l1_y1 + dy

        # Plot the line using the point and end coordinates
        axis.plot([l1_x1, x_end], [l1_y1, y_end], 'g-')

        x_end = l2_x1 + dx
        y_end = l2_y1 + dy

        axis.plot([l2_x1, x_end], [l2_y1, y_end], 'g-')
       
    
    def distance_between_points(self, x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.

        Args:
            x1: x-coordinate of the first point.
            y1: y-coordinate of the first point.
            x2: x-coordinate of the second point.
            y2: y-coordinate of the second point.

        Returns:
            The Euclidean distance between the two points.
        """
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
        
    def contains_point(self, x, y):
        slope = self.l1.slope

        if slope >= 16331230000000000:
            # Es perpendicular al eje x
            l = Line(Point(0, y), Point(1, y))
            y1 = l.intersection(self.l1)
            y2 = l.intersection(self.l2)
            
            y1 = float(y1[0][0])
            y2 = float(y2[0][0])
        else:
            l = Line(Point(x, 0), Point(x, 1))
            y1 = l.intersection(self.l1)
            y2 = l.intersection(self.l2)
        
            y1 = float(y1[0][1])
            y2 = float(y2[0][1])

        if (y1 > 0 and y2 < 0) or (y1 < 0 and y2 > 0):
            return True
        else:
            return False

        

    
    
class CircularTrajectory(ObstacleTrajectory):
    def __init__(self, point1, point2, position, velocity) -> None:
        x1, y1 = point1
        x2, y2 = point2
        v, w = velocity
        x, y, th = position

        self.radius = v/w

        self.center_x = x + self.radius * math.cos(th + math.pi/2)
        self.center_y = y + self.radius * math.sin(th + math.pi/2)

        radius_1 = ((self.center_x - x1) ** 2 + (self.center_y - y1) ** 2) ** 0.5
        radius_2 = ((self.center_x - x2) ** 2 + (self.center_y - y2) ** 2) ** 0.5  

        l1 = Circle(Point(self.center_x, self.center_y), radius_1)
        l2 = Circle(Point(self.center_x, self.center_y), radius_2)

        super().__init__(l1, l2)
    
    def plot(self, axis):
        axis.add_patch(plt.Circle((self.l1.center.coordinates[0], self.l1.center.coordinates[1]), self.l1.radius, color='green', fill=False))
    
        axis.add_patch(plt.Circle((self.l2.center.coordinates[0], self.l2.center.coordinates[1]), self.l2.radius, color='green', fill=False))    


    def distance_between_points(self, x1, y1, x2, y2):
        #Engordar el punto
        p = Circle(Point(x2, y2), 0.1)
        inter = self.l1.intersection(p)
        
        if inter == []:
            radius = self.l2.radius
        else:
            radius = self.l1.radius
            
        d = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        theta = 2 * math.atan(d / (2 * radius))
        arclength = radius * theta
        return arclength
    

    def contains_point(self, x, y):
        if abs(self.l1.radius) < abs(self.l2.radius):
            r1 = self.l1.radius
            r2 = self.l2.radius
        else:
            r1 = self.l2.radius
            r2 = self.l1.radius
        #((robot_x - Cx1)^2 + (robot_y - Cy1)^2) > R1^2 && ((robot_x - Cx2)^2 + (robot_y - Cy2)^2) < R2^2
        if (pow(x - self.center_x, 2) + pow(y - self.center_y, 2)) > pow(r1, 2) and (pow(x - self.center_x, 2) + pow(y - self.center_y, 2)) < pow(r2, 2):
            return True
        else:
            return False
        
        

    
class RobotTrajectory():
    def __init__(self, radius, position) -> None:
        self.radius = radius
        x, y, th = position
        center_x = x + radius * math.cos(th + math.pi/2)
        center_y = y + radius * math.sin(th + math.pi/2)
        self.c = Circle(Point(center_x, center_y), radius)

    def plot(self, axis):
        axis.add_patch(plt.Circle((self.c.center.coordinates[0], self.c.center.coordinates[1]), self.c.radius, color='blue', fill=False))
