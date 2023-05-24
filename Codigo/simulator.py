import matplotlib.pyplot as plt
import numpy as np
import matplotlib.lines as mlines
import matplotlib.animation as animation

import DOVS.dovs as DOVS

class DynamicObstacle:
    def __init__(self, v, w, x, y, theta, radius) -> None:
        self.v = v
        self.w = w
        self.x = x
        self.y = y
        self.theta = theta
        self.radius = radius

class Robot(DynamicObstacle):
    # Añadir aceleraciones
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

timestep = 0.2


def computeDOVS(robot, obstacles, timestep, fig_dovs, ax_dovs):
    print("Here the DOVS function should be called")
    # return linear and angular velocities chosen for the robot
    dovs = DOVS.DOVS(robot, obstacles, timestep, fig_dovs, ax_dovs)
    v, w = dovs.compute_DOVS()
    return v, w

plt.close('all')
fig, axis = plt.subplots(2,2, figsize=(12,12))
ax = axis[1, :]
ax_dovs = (axis[0, 0], axis[0, 1])

# Remove unused axis
ax[1].remove()

# Adjust spacing between subplots
fig.subplots_adjust(hspace=0.3)

ax = ax[0]



# ax = ax.reshape((-1))
robot = Robot(0.0, 0.0, 0.0, 2.0, -np.pi/2, 0.2, 0.0, -2.0, 0.0, 0.7, -np.pi/2, np.pi/2, 0.7, np.pi/2)
obstacles_vec = []
obstacles_vec.append(DynamicObstacle(0.5, 0.1, 0.0, 2.0, 0.0, 0.2))
#obstacles_vec.append(DynamicObstacle(0.5/5, 0.1/5, 0.0, 0.0, 0.0, 0.2))


obstacles_artist = []
obstacles_arrow_artist = []
for i_obs in range(len(obstacles_vec)):
    obstacle = plt.Circle((obstacles_vec[i_obs].x, obstacles_vec[i_obs].y), obstacles_vec[i_obs].radius, color='black', fill=True)
    ax.add_artist(obstacle)
    obstacles_artist.append(obstacle)
    obstacles_arrow_artist.append(plt.arrow(obstacles_vec[i_obs].x, obstacles_vec[i_obs].y, obstacles_vec[i_obs].radius*np.cos(obstacles_vec[i_obs].theta), obstacles_vec[i_obs].radius*np.sin(obstacles_vec[i_obs].theta), width=0.035, color="red"))
goal_artist = mlines.Line2D([robot.x_goal], [robot.y_goal],
                    color='red', marker='*', linestyle='None',
                    markersize=15, label='Goal')

ax.title.set_text(r'Time: {0:.2f} s'.format(0))
robot_artist = plt.Circle((robot.x, robot.y), robot.radius, fill=True, color="blue")
robot_arrow_artist = plt.arrow(robot.x, robot.y, robot.radius*np.cos(robot.theta), robot.radius*np.sin(robot.theta), width=0.035, color="red")
# This is for plotting the trajectory
# ax.add_artist(plt.Circle((robot.x, robot.y), robot.radius/8, fill=True, color="blue"))

ax.add_artist(goal_artist)
ax.add_artist(robot_artist)
ax.legend([robot_artist, obstacle, goal_artist], ['Robot', 'Obstacle', 'Goal'], loc='lower right', markerscale=0.4)
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)
# ax.axis('equal')
# plt.tight_layout()

#fig_dovs, ax_dovs = plt.subplots(1, 3, figsize=(19,12))
#cid = fig_dovs.canvas.mpl_connect('button_press_event', onclick)
    
time = 0
def update(i_video):
    global robot_arrow_artist
    time = i_video*timestep
    for axis in ax_dovs:
        axis.clear()
    v_new, w_new = computeDOVS(robot=robot, obstacles=obstacles_vec, timestep=timestep, fig_dovs=fig, ax_dovs=ax_dovs)
    plt.pause(0.2)
    robot.v = v_new
    robot.w = w_new

    for i_obs in range(len(obstacles_vec)):
        if abs(obstacles_vec[i_obs].x)> 2.0:
            obstacles_vec[i_obs].theta = (obstacles_vec[i_obs].theta + np.pi)%(2*np.pi)
        obstacles_vec[i_obs].x = obstacles_vec[i_obs].x + obstacles_vec[i_obs].v*np.cos(obstacles_vec[i_obs].theta)*timestep
        obstacles_vec[i_obs].y = obstacles_vec[i_obs].y + obstacles_vec[i_obs].v*np.sin(obstacles_vec[i_obs].theta)*timestep
        obstacles_vec[i_obs].theta = obstacles_vec[i_obs].theta + obstacles_vec[i_obs].w*timestep
        obstacles_artist[i_obs].center = (obstacles_vec[i_obs].x, obstacles_vec[i_obs].y)
        obstacles_arrow_artist[i_obs].remove()
        obstacles_arrow_artist[i_obs] = plt.arrow(obstacles_vec[i_obs].x, obstacles_vec[i_obs].y, obstacles_vec[i_obs].radius*np.cos(obstacles_vec[i_obs].theta), obstacles_vec[i_obs].radius*np.sin(obstacles_vec[i_obs].theta), width=0.035, color="red")


    
    robot.x = robot.x + robot.v*np.cos(robot.theta)*timestep
    robot.y = robot.y + robot.v*np.sin(robot.theta)*timestep
    robot.theta = robot.theta + robot.w*timestep
    robot_artist.center = (robot.x, robot.y)
    # robot_arrow_artist.center
    robot_arrow_artist.remove()
    robot_arrow_artist = plt.arrow(robot.x, robot.y, robot.radius*np.cos(robot.theta), robot.radius*np.sin(robot.theta), width=0.035, color="red")
    # This is for plotting the trajectory
    # ax.add_artist(plt.Circle((robot.x, robot.y), robot.radius/8, fill=True, color="blue"))
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    # ax.axis('equal')
    ax.title.set_text(r' -- Time: {0:.2f} s'.format(time))

anim = animation.FuncAnimation(fig, update, interval=0.2 * 1000)
anim.running = True
# ffmpeg_writer = animation.FFMpegWriter(fps=4, metadata=dict(artist='Me'), bitrate=1800)
# anim.save("saved_experiment.mp4", writer=ffmpeg_writer)

plt.show()