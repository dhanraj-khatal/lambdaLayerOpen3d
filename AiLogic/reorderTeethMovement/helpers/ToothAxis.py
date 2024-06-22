import numpy as np

from .functions import get_slope


class ToothAxis:
    def __init__(self, first_point, second_point):
        self.first_point = None
        self.second_point = None
        self.slope = None
        self.update_axis(first_point, second_point)

    def get_slope(self):
        return self.slope

    def update_axis(self, first_point, second_point):
        self.first_point = np.asarray(first_point)
        self.second_point = np.asarray(second_point)

        self.slope = get_slope(points=[first_point, second_point])

    def get_second_point(self):
        return self.second_point

    def to_dict(self):
        return {
            'slope': self.slope,
            'first_point': self.first_point,
            'second_point': self.second_point,
        }
