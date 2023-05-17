import matplotlib.patches as patches

#TODO: Es buen nombre??
class DOV():
    #dynamic_object_velocity
    def __init__(self, velocity_time_space = []) -> None:
        passBehind = []
        passFront = []
    
        # print(velocity_time_space)
        if len(velocity_time_space) != 0:
            #TODO:Añadir que guarde los tiempos
            passBehind, passFront = [(passBehind[0][1], passBehind[0][2]) for passBehind in velocity_time_space], [(passFront[1][1], passFront[1][2]) for passFront in velocity_time_space]
            tiempos = [passBehind[0][0] for passBehind in velocity_time_space]
            print("tiempos")
            print(tiempos)
            vertices = passBehind + passFront[::-1]
            self.dov = patches.Polygon(vertices, closed=True, facecolor='green')
        else:
            self.dov = patches.Polygon([(0,0),(0,0)], closed=True, facecolor='green')

        self.passBehind = passBehind
        self.passFront = passFront

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
