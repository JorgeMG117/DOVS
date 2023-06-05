import numpy as np
from tests import DynamicObstacle, Robot, Test


tests_list = [
    Test(
        robot = Robot(
            v = 0.667,
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
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.1, 
                w = 0, 
                x = 1.0, 
                y = 1.0, 
                theta = -2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5, 
                w = 0.1, 
                x = -2.0, 
                y = 0.0, 
                theta = 0.0, 
                radius = 0.2)
        ],
        colision = False,
        name = "test_0",
        description = "Llega a la meta pasando por delante de los dos obstaculos",
        tiempo_fin = -1
    ),

    Test(
        robot = Robot(
            v = 0.52,
            w = -0.1,
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
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.1, 
                w = 0, 
                x = 1.0, 
                y = 1.0, 
                theta = -2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5, 
                w = 0.1, 
                x = -2.0, 
                y = 0.0, 
                theta = 0.0, 
                radius = 0.2)
        ],
        colision = True,
        name = "test_1",
        description = "Mismo mapa que el test 0, choca con el que va recto"
    ),

    Test(
        robot = Robot(
            v = 0.14,
            w = 0.037,
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
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.1, 
                w = 0, 
                x = 1.0, 
                y = 1.0, 
                theta = -2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5, 
                w = 0.1, 
                x = -2.0, 
                y = 0.0, 
                theta = 0.0, 
                radius = 0.2)
        ],
        colision = True,
        name = "test_2",
        description = "Mismo mapa que el test 0, choca con el que esta girando"
    ),

    Test(
        robot = Robot(
            v = 0.3,
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
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.5, 
                w = 0, 
                x = -2.0, 
                y = 0.0, 
                theta = 0.0, 
                radius = 0.2),
        ],
        colision = True,
        name = "test_3",
        description = "Llega a la meta pasando despues del obstaculo",
        tiempo_fin = -1
    ),

    Test(
        robot = Robot(
            v = 0.7,
            w = -0.04,
            x = -2.0, 
            y = 2.0, 
            theta = -np.pi/4, 
            radius = 0.2, 
            x_goal = 1.0, 
            y_goal = -2.0, 
            min_v = 0.0, 
            max_v = 0.7, 
            min_w = -np.pi/2, 
            max_w = np.pi/2, 
            max_av = 0.7, 
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.25, 
                w = 0, 
                x = 0.0, 
                y = 2.0, 
                theta = -2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5/1.3, 
                w = 0.1/1.3, 
                x = -2.0, 
                y = 0.0, 
                theta = np.pi/4, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5, 
                w = 0.05, 
                x = 2.0, 
                y = -1.0, 
                theta = np.pi, 
                radius = 0.2)
        ],
        colision = False,
        name = "test_4",
        description = "Llega a la meta, pasa antes de tres obstaculos",
        tiempo_fin = -1
    ),

    Test(
        robot = Robot(
            v = 0.1,
            w = 0.0,
            x = -2.0, 
            y = 2.0, 
            theta = -np.pi/4, 
            radius = 0.2, 
            x_goal = 1.0, 
            y_goal = -2.0, 
            min_v = 0.0, 
            max_v = 0.7, 
            min_w = -np.pi/2, 
            max_w = np.pi/2, 
            max_av = 0.7, 
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.25, 
                w = 0, 
                x = 0.0, 
                y = 2.0, 
                theta = -2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5/1.3, 
                w = 0.1/1.3, 
                x = -2.0, 
                y = 0.0, 
                theta = np.pi/4, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5, 
                w = 0.05, 
                x = 2.0, 
                y = -1.0, 
                theta = np.pi, 
                radius = 0.2)
        ],
        colision = True,
        name = "test_5",
        description = "Pasar despues de tres obstaculos",
        tiempo_fin = 8
    ),

    Test(
        robot = Robot(
            v = 0.5,
            w = 0.14,
            x = -2.0, 
            y = -2.0, 
            theta = 0,
            radius = 0.2, 
            x_goal = 1.6, 
            y_goal = 2.0, 
            min_v = 0.0, 
            max_v = 0.7, 
            min_w = -np.pi/2, 
            max_w = np.pi/2, 
            max_av = 0.7, 
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.35, 
                w = 0.05, 
                x = 1.0, 
                y = -2.0, 
                theta = 2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.33/1.2, 
                w = -0.05/1.2, 
                x = -2.0, 
                y = 1.4, 
                theta = 0, 
                radius = 0.2)
                
        ],
        colision = False,
        name = "test_6",
        description = "Pasar por detras de un obstaculo y por delante de otro, llega la meta haciendo un giro",
        tiempo_fin = -1
    ),

    Test(
        robot = Robot(
            v = 0.1,
            w = 0.0,
            x = 0.0, 
            y = 0.0, 
            theta = 0,
            radius = 0.2, 
            x_goal = 2.0, 
            y_goal = 0.0, 
            min_v = 0.0, 
            max_v = 0.7, 
            min_w = -np.pi/2, 
            max_w = np.pi/2, 
            max_av = 0.7, 
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.35, 
                w = 0.0, 
                x = -2.0, 
                y = 0.0, 
                theta = 0, 
                radius = 0.2) 
        ],
        colision = True,
        name = "test_7",
        description = "Si no lleva suficiente velocidad choca con obstaculo que va detras",
        tiempo_fin = -1
    ),

    Test(
        robot = Robot(
            v = 0.5,
            w = 0.0,
            x = -2.0, 
            y = 2.0, 
            theta = -np.pi/4,
            radius = 0.2, 
            x_goal = 2, 
            y_goal = -2.0, 
            min_v = 0.0, 
            max_v = 0.7, 
            min_w = -np.pi/2, 
            max_w = np.pi/2, 
            max_av = 0.7, 
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.25, 
                w = 0, 
                x = 0.0, 
                y = 2.0, 
                theta = -np.pi/2, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.25/1.2, 
                w = 0.05/1.2, 
                x = 2.0, 
                y = 2.0, 
                theta = -2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.25, 
                w = -0.02, 
                x = -1.0, 
                y = 1.0, 
                theta = -2*np.pi/3, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.5, 
                w = 0, 
                x = -1.0, 
                y = -1.0, 
                theta = np.pi/4, 
                radius = 0.2),
            DynamicObstacle(
                v = 0.2, 
                w = -0.03, 
                x = 2.0, 
                y = -1.0, 
                theta = -2*np.pi/3-0.5, 
                radius = 0.2)
                
        ],
        colision = False,
        name = "test_8",
        description = "Llega a la meta con 5 obstaculos",
        tiempo_fin = -1
    ),

    Test(
        robot = Robot(
            v = 0.667,
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
            max_aw = np.pi/2),

        obstacles = [
            DynamicObstacle(
                v = 0.5, 
                w = 0, 
                x = 0.5, 
                y = -1.0, 
                theta = 2*np.pi/3, 
                radius = 0.2)
        ],
        colision = True,
        name = "test_9",
        description = "Colisiona con un obstaculo de frente",
        tiempo_fin = -1
    )
    
]