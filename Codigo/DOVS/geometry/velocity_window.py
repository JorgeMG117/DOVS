import matplotlib.patches as patches

class VelocityWindow():
    def __init__(self, robot, timestep) -> None:
        # poly = Polygon([], fill=False)
        #a = (vf - vi) / t
        v_max = robot.max_av * timestep + robot.v
        v_min = -robot.max_av * timestep + robot.v
        w_max = robot.max_aw * timestep + robot.v
        w_min = -robot.max_aw * timestep + robot.v

        self.velocity_window = patches.Polygon(list(zip([v_max, robot.v, v_min, robot.v], [robot.w, w_max, robot.w, w_min])), color='lightblue', alpha=0.5, fill=True)

    def contains(self, point):
        return self.velocity_window.contains_point(point)

    def plot(self, axis):
        axis.add_patch(self.velocity_window)
