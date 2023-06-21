# DOVS

Este repositorio contiene la implementación en Python del modelo DOVS para la navegación en entornos dinámicos, definido en (María Teresa, Eduardo, Luis, 2018)


[![Alt text](https://img.youtube.com/vi/m3_k_ppJ7_Y/0.jpg)](https://www.youtube.com/watch?v=m3_k_ppJ7_Y)

## Uso

Primero se tiene que instalar los paquetes necesarios, para ello ejecutar:

```
pip install -r requirements.txt
```

Para probar la librería DOVS se puede ejecutar uno de los programas principales, con:

```
python simulator.py
```

```
python interactive.py
```

```
python tests.py
```

La parte del código que define el entorno y sus características, el robot y los obstáculos, es la siguiente:

```python
robot = Robot(
            v = 0.66,
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
            max_aw = np.pi/2)
 
obstacles_vec.append(
    DynamicObstacle(
            v = 0.25, 
            w = 0, 
            x = 0.0, 
            y = 2.0, 
            theta = -2*np.pi/3, 
            radius = 0.2))
obstacles_vec.append(
    DynamicObstacle(
            v = 0.5, 
            w = 0.1, 
            x = -2.0, 
            y = 0.0, 
            theta = np.pi/4, 
            radius = 0.2))
```

Modificando estas líneas de código, pueden conseguirse diferentes configuraciones de entorno.


## Referencias

María Teresa Lorente, Eduardo Owen, and Luis Montano. Model-based robocentric planning and navigation for dynamic environments. The International Journal of Robotics Research, 37(8):867–889, 2018.
