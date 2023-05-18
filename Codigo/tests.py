
# Leer fichero de configuracion del dovs,

# Recorrer los casos de prueba ejecutando
    # Guardar imagen

# Mostrar todas las imagenes

from matplotlib import pyplot as plt
import numpy as np

from DOVS.dovs import DOVS

class DynamicObstacle:
    def __init__(self, v, w, x, y, theta, radius) -> None:
        self.v = v
        self.w = w
        self.x = x
        self.y = y
        self.theta = theta
        self.radius = radius

class Robot(DynamicObstacle):
    def __init__(self, v, w, x, y, theta, radius, x_goal, y_goal, min_v, max_v, min_w, max_w, max_av, max_aw) -> None:
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.min_v = min_v
        self.max_v = max_v
        self.min_w = min_w
        self.max_w = max_w
        self.max_av = max_av
        self.max_aw = max_aw
        super().__init__(v, w, x, y, theta, radius)

class Test:
    def __init__(self, robot, obstacles, name, description = "") -> None:
        self.robot = robot
        self.obstacles = obstacles
        self.name = name
        self.description = description

robot1 = Robot(
        v = 0.0, 
        w = 0.0, 
        x = 0.0, 
        y = 2.0, 
        theta = -np.pi/2, 
        radius = 0.2, 
        x_goal = 0.0, 
        y_goal = -2.0, 
        min_v = 0.0, 
        max_v = 0.7, 
        min_w = -np.pi/2, 
        max_w = np.pi/2, 
        max_av = 0.7, 
        max_aw = np.pi/2)

tests_list = [
    Test(
        robot = robot1,
        obstacles = [
            DynamicObstacle(
                v = 0.1, 
                w = 0, 
                x = -3.0, 
                y = 2.0, 
                theta = -np.pi/4, 
                radius = 0.2)
        ],
        name = "test_1"
    ),
    Test(
        robot = robot1,
        obstacles = [
            DynamicObstacle(
                v = 0.1, 
                w = 0, 
                x = -3.0, 
                y = 2.0, 
                theta = -np.pi/4, 
                radius = 0.2)
        ],
        name = "test_2"
    ),
    Test(
        robot = robot1,
        obstacles = [
            DynamicObstacle(
                v = 0.1, 
                w = 0, 
                x = -3.0, 
                y = 2.0, 
                theta = -np.pi/4, 
                radius = 0.2)
        ],
        name = "test_3"
    )
    
]

def onclick(event):
    print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          ('double' if event.dblclick else 'single', event.button,
           event.x, event.y, event.xdata, event.ydata))
    print(event.inaxes)
    


def main():
    timestep = 0.2
    fig, ax = plt.subplots(1, 2, figsize=(19,12))
    for test in tests_list:
        #plt.ion()
        

        
        # plt.title('Some Title')
        # plt.xlabel('Year')
        # plt.ylabel('Some measurements')
        
        dovs = DOVS(test.robot, test.obstacles, timestep, fig, ax)
        dovs.compute_DOVS()
        
        plt.savefig('imagenes/' + test.name + '.png')
        # plt.close()
        #cid = fig.canvas.mpl_connect('button_press_event', onclick)
    
        # plt.draw()
        # plt.pause(0.2)
        #plt.show()

        #input("Press enter...")
        # plt.clf()
        # #fig.canvas.flush_events()

        #fig.tight_layout() No
        
        for axis in ax:
            axis.clear()

            
        
        #plt.ioff()  # Turn off interactive mode
        #plt.close(fig)
        


if __name__ == "__main__":
    main()