# DOVS

This repository contains the Python implementation of the DOVS model for navigation in dynamic environments, defined in (Maria Teresa, Eduardo, Luis, 2018).


[![Alt text](https://img.youtube.com/vi/m3_k_ppJ7_Y/0.jpg)](https://www.youtube.com/watch?v=m3_k_ppJ7_Y)

***Language***
- [🇪🇸 Español](./README.es.md)
- 🇺🇸 English

## Usage

First you have to install the necessary packages, to do this run:

```
pip install -r requirements.txt
```

To test the DOVS library you can run one of the main programs, with:

```
python simulator.py
```

```
python interactive.py
```

```
python tests.py
```

The part of the code that defines the environment and its features, the robot and the obstacles, is as follows:

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

By modifying these lines of code, different environment configurations can be achieved.


## References

María Teresa Lorente, Eduardo Owen, and Luis Montano. Model-based robocentric planning and navigation for dynamic environments. The International Journal of Robotics Research, 37(8):867–889, 2018.
