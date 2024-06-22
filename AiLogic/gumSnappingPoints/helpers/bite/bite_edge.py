from dataclasses import dataclass

import numpy as np
from open3d.cpu.pybind.geometry import LineSet

from .bite_vertex import BiteVertex
from ..functions import *


@dataclass(unsafe_hash=True)
class BiteEdge:
    """
    Description:
        This class defines an edge from bite vertex a to bite vertex b
    """
    from_vertex: BiteVertex = BiteVertex()
    to_vertex: BiteVertex = BiteVertex()

    slope: np.array = None
    line_set: LineSet = None
    other_jaw: bool = False
    same_jaw: bool = False

    to_world_center: bool = False

    def __post_init__(self):
        v1 = self.from_vertex.get_vertex()
        v2 = self.to_vertex.get_vertex()

        self.slope = get_slope([
            v1,
            v2
        ])

        self.line_set = get_line_segment_between_points(
            v1,
            v2
        )

    def get_edge_slope(self):
        return self.slope

    def get_line_set(self):
        return self.line_set
