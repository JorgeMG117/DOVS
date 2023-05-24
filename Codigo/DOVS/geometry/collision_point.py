import matplotlib.patches as patches

class CollisionPoint():
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def plot(self, axis):
        axis.plot(self.x, self.y, 'k.')
