import matplotlib.patches as patches
import numpy as np

class CollisionPoint():
    def __init__(self, x, y, trajectory_radius) -> None:
        self.x = x
        self.y = y
        
        self.angle = np.arctan2(2*self.x*self.y, pow(self.x,2)-pow(self.y,2))
        self.arclength = trajectory_radius * self.angle#Distancia desde el robot al punto de colision

        theta = np.arctan2(2*self.x*abs(self.y), pow(self.x,2)-pow(abs(self.y),2))
        theta = (theta + 2*np.pi) % (2*np.pi)
        arclength = trajectory_radius * theta
        self.angle_arc = (arclength/trajectory_radius) * (180 / np.pi) #Angulo en grados sobre la trayectoria del robot, desde el robot al punto de colision

        #Quiza añadir una variable que sea en uso o algo asi, y que cuando en velocidad te salga una distancia negativa la deshabilitas
    def plot(self, axis):
        axis.plot(self.x, self.y, 'k.')
