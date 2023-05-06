import matplotlib.patches as patches

#TODO: Es buen nombre??
class DOV():
    #dynamic_object_velocity
    def __init__(self, velocity_time_space) -> None:
        #TODO:Añadir que guarde los tiempos
        passBehind, passFront = [(passBehind[0][1], passBehind[0][2]) for passBehind in velocity_time_space], [(passFront[1][1], passFront[1][2]) for passFront in velocity_time_space]

        vertices = passBehind + passFront[::-1]

        self.dov = patches.Polygon(vertices, closed=False, facecolor='green')

    def contains(self, point):
        return self.dov.contains_point(point)

    def plot(self, axis):
        axis.add_patch(self.dov)
        # for point in passBehind:
        #     # print(point)
        #     # input("Press enter to continue...")
        #     ax.plot(point[0], point[1], "k.")
        #     # fig.canvas.draw()   # Redraw the plot
        #     # plt.pause(0.5)
            
        # for point in passFront:
        #     # print(point)
            
        #     # input("Press enter to continue...")
        #     ax.plot(point[0], point[1], "b.")
        #     # fig.canvas.draw()   # Redraw the plot
        #     # plt.pause(0.5)
