import shapely.geometry as sg
import shapely.ops as so

class DOV():
    #dynamic_object_velocity
    def __init__(self, velocity_time_space = []) -> None:
        if len(velocity_time_space) <= 1: self.valid = False; return
        else: self.valid = True
            
        self.passBehind,  self.passFront = [(passBehind[0][1], passBehind[0][2]) for passBehind in velocity_time_space], [(passFront[1][1], passFront[1][2]) for passFront in velocity_time_space]
        self.passBehind_times,  self.passFront_times = [value[0][0] for value in velocity_time_space], [value[1][0] for value in velocity_time_space]
        
        vertices = self.passBehind + self.passFront[::-1]
        
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
        
        if self.dov.is_valid and dovs.dov.is_valid:
            self.dov = so.unary_union([self.dov, dovs.dov])
            if self.dov.geom_type == 'Polygon':
                self.dov = sg.MultiPolygon([self.dov])
        else:
            geometries = []
            for geom in self.dov.geoms: 
                geometries.append(geom)
            for geom in dovs.dov.geoms: 
                geometries.append(geom)
            
            self.dov = sg.MultiPolygon(geometries)
            
        self.passBehind = self.passBehind + dovs.passBehind
        self.passFront = self.passFront + dovs.passFront
        self.passBehind_times = self.passBehind_times + dovs.passBehind_times
        self.passFront_times = self.passFront_times + dovs.passFront_times



    def plot(self, axis):
        if not self.valid: return
        
        for geom in self.dov.geoms:    
            xs, ys = geom.exterior.xy
            axis.fill(xs, ys, alpha=0.5, fc='r', ec='none')

        for point in self.passBehind:
            axis.plot(point[0], point[1], "k.")

        for point in self.passFront:
            axis.plot(point[0], point[1], "b.")
