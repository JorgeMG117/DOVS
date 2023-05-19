import matplotlib.patches as patches

class VelocityWindow():
    def __init__(self, robot, timestep) -> None:
        # poly = Polygon([], fill=False)
        #a = (vf - vi) / t
        v_max = robot.max_av * timestep + robot.v
        v_min = -robot.max_av * timestep + robot.v
        w_max = robot.max_aw * timestep + robot.w
        w_min = -robot.max_aw * timestep + robot.w

        self.velocity_window = patches.Polygon(list(zip([robot.w, w_max, robot.w, w_min], [v_max, robot.v, v_min, robot.v])), color='lightblue', alpha=0.5, fill=True)
        self.v = robot.v
        self.w = robot.w

    def contains(self, point):
        return self.velocity_window.contains_point(point)

    def plot(self, axis):
        axis.add_patch(self.velocity_window)
        axis.plot(self.w, self.v, 'r.')
