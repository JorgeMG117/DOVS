import matplotlib.patches as patches

#TODO: Es buen nombre??
class DOV():
    #dynamic_object_velocity
    def __init__(self, velocity_time_space = []) -> None:
        passBehind = []
        passFront = []
        passBehind_times = []
        passFront_times = []
    
        # print(velocity_time_space)
        if len(velocity_time_space) != 0:
            
            passBehind, passFront = [(passBehind[0][1], passBehind[0][2]) for passBehind in velocity_time_space], [(passFront[1][1], passFront[1][2]) for passFront in velocity_time_space]
            passBehind_times, passFront_times = [value[0][0] for value in velocity_time_space], [value[1][0] for value in velocity_time_space]
            
            vertices = passBehind + passFront[::-1]
            self.dov = patches.Polygon(vertices, closed=True, facecolor='green')
        else:
            self.dov = patches.Polygon([(0,0),(0,0)], closed=True, facecolor='green')

        self.passBehind = passBehind
        self.passFront = passFront
        self.passBehind_times = passBehind_times
        self.passFront_times = passFront_times
        print("self.passBehind_times")
        print(self.passBehind_times)
        print("self.passFront_times")
        print(self.passFront_times)

    def contains(self, point):
        return self.dov.contains_point(point)
    
    def combine_DOVS(self, dovs):
        return dovs

    def plot(self, axis):
        axis.add_patch(self.dov)

        print("len(self.passBehind)")
        print(len(self.passBehind))
        for point in self.passBehind:
            print(point)
            # input("Press enter to continue...")
            axis.plot(point[0], point[1], "k.")
            # fig.canvas.draw()   # Redraw the plot
            # plt.pause(0.5)
            
        print("len(self.passFront)")
        print(len(self.passFront))
        for point in self.passFront:
            print(point)
            
            # input("Press enter to continue...")
            axis.plot(point[0], point[1], "b.")
            # fig.canvas.draw()   # Redraw the plot
            # plt.pause(0.5)
