import matplotlib.patches as patches

import shapely.geometry as sg
import shapely.ops as so

#TODO: Es buen nombre??
class DOV():
    #dynamic_object_velocity
    def __init__(self, velocity_time_space = []) -> None:

        if len(velocity_time_space) == 0: self.valid = False; return
        else: self.valid = True
            
        self.passBehind,  self.passFront = [(passBehind[0][1], passBehind[0][2]) for passBehind in velocity_time_space], [(passFront[1][1], passFront[1][2]) for passFront in velocity_time_space]
        self.passBehind_times,  self.passFront_times = [value[0][0] for value in velocity_time_space], [value[1][0] for value in velocity_time_space]
        
        vertices = self.passBehind + self.passFront[::-1]
        #self.dov = patches.Polygon(vertices, closed=True, facecolor='green')
        
        p = sg.Polygon(vertices)
        self.dov = sg.MultiPolygon([p])


    # def contains(self, point):
    #     if not self.valid: return False
    #     return self.dov.contains_point(point)

    
    def combine_DOVS(self, dovs):
        if not dovs.valid: return
        if not self.valid:
            self.valid = dovs.valid
            self.dov = dovs.dov
            self.passBehind = dovs.passBehind
            self.passFront = dovs.passFront
            self.passBehind_times = dovs.passBehind_times
            self.passFront_times = dovs.passFront_times
            return
        
        self.dov = so.unary_union([self.dov, dovs.dov])
        if self.dov.geom_type == 'Polygon':
            self.dov = sg.MultiPolygon([self.dov])
        
        self.passBehind = self.passBehind + dovs.passBehind
        self.passFront = self.passFront + dovs.passFront
        self.passBehind_times = self.passBehind_times + dovs.passBehind_times
        self.passFront_times = self.passFront_times + dovs.passFront_times



    def plot(self, axis):
        print(self.valid)
        if not self.valid: return
        #axis.add_patch(self.dov)

        #xs, ys = self.dov.exterior.xy
        #axis.fill(xs, ys, alpha=0.5, fc='r', ec='none')
        
        for geom in self.dov.geoms:    
            xs, ys = geom.exterior.xy
            axis.fill(xs, ys, alpha=0.5, fc='r', ec='none')

        # print("self.passBehind_times")
        # print(self.passBehind_times)
        # print("self.passFront_times")
        # print(self.passFront_times)

        print("len(self.passBehind)")
        print(len(self.passBehind))
        for point in self.passBehind:
            # print(point)
            # input("Press enter to continue...")
            axis.plot(point[0], point[1], "k.")
            # fig.canvas.draw()   # Redraw the plot
            # plt.pause(0.5)
            
        print("len(self.passFront)")
        print(len(self.passFront))
        for point in self.passFront:
            # print(point)
            
            # input("Press enter to continue...")
            axis.plot(point[0], point[1], "b.")
            # fig.canvas.draw()   # Redraw the plot
            # plt.pause(0.5)
