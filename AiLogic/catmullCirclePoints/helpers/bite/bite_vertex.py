import numpy as np

from dataclasses import dataclass


@dataclass(unsafe_hash=True)
class BiteVertex:
    """
    Description:
        This class defines the vertex of a bite graph
    """
    tooth_id: int = -1
    coordinate: np.array = np.array((0, 0, 0))

    is_center: bool = False
    is_top: bool = False
    is_front: bool = False
    is_origin: bool = False

    def get_vertex(self):
        return self.coordinate
