import numpy as np

class Body:
    def __init__(self, mass, position, velocity, color, index=None, radius=1.0):
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.color = color
        self.index = index
        self.radius = radius

    def to_dict(self):
        return {'pos': self.position,'mass': self.mass,'index': self.index,'radius': self.radius}