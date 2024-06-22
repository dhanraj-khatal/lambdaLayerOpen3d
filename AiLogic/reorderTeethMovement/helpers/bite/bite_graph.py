from typing import List

import numpy as np

from ..constants import *
from .bite_vertex import *
from .bite_edge import *


class BiteGraph(object):

    def __init__(self, teeth_objects):
        self.vertices: List = [{"center": None, "top": None, "front": None} for _ in range(32)]
        self.edges: List = [{"center": None, "top": None, "front": None} for _ in range(32)]

        self.teeth_count = len(teeth_objects)
        self.create_graph(teeth_objects)

    def create_graph(self, teeth_objects):
        # add vertices
        for i, sphere in enumerate(teeth_objects):
            if sphere.is_dummy:
                sphere.center_bite_vertex = BiteVertex(is_origin=True, is_center=True)
                sphere.top_bite_vertex = BiteVertex(is_origin=True, is_top=True)
                sphere.front_bite_vertex = BiteVertex(is_origin=True, is_front=True)

                self.vertices[i] = {
                    "center": sphere.center_bite_vertex,
                    "top": sphere.center_bite_vertex,
                    "front": sphere.center_bite_vertex,
                }

                continue

            center = sphere.tooth.get_center()
            sphere.center_bite_vertex = BiteVertex(
                tooth_id=i+1,
                coordinate=center,
                is_center=True
            )

            if sphere.root_face:
                sphere.top_bite_vertex = BiteVertex(
                    tooth_id=i,
                    coordinate=sphere.root_face["face_center"],
                    is_top=True
                )

            if sphere.front_face:
                sphere.front_bite_vertex = BiteVertex(
                    tooth_id=i,
                    coordinate=sphere.front_face["face_center"],
                    is_front=True
                )

            self.vertices[i] = {
                "center": sphere.center_bite_vertex,
                "top": sphere.top_bite_vertex,
                "front": sphere.front_bite_vertex,
            }

        # add edges
        for i, sphere in enumerate(teeth_objects):
            if sphere.is_dummy:
                continue

            for n, neighbours in enumerate([sphere.other_jaw_neighbours, sphere.same_jaw_neighbours]):
                other_jaw = (n == 0)
                for tooth_id in neighbours:
                    j = tooth_id - 1
                    edge = BiteEdge(
                        from_vertex=sphere.center_bite_vertex,
                        to_vertex=self.vertices[j]["center"],
                        to_world_center=self.vertices[j]["center"].is_origin,
                        other_jaw=other_jaw,
                    )

                    sphere.center_bite_edges[j] = edge

                    edge = BiteEdge(
                        from_vertex=sphere.top_bite_vertex,
                        to_vertex=self.vertices[j].top,
                        to_world_center=self.vertices[j].top.is_origin,
                        other_jaw=other_jaw,
                    )

                    sphere.top_bite_edges[j] = edge

                    edge = BiteEdge(
                        from_vertex=sphere.front_bite_vertex,
                        to_vertex=self.vertices[j].front,
                        to_world_center=self.vertices[j].front.is_origin,
                        other_jaw=other_jaw,
                    )

                    sphere.front_bite_edges[j] = edge

            self.edges[i].center = sphere.center_bite_edges
            self.edges[i].top = sphere.top_bite_edges
            self.edges[i].front = sphere.front_bite_edges

    def draw_center_graph(self):
        points = []
        line_segments = []

        for vertex in self.vertices:
            if vertex["center"].is_origin:
                continue

            points.append(vertex["center"].get_vertex())

        for edges in self.edges:
            for edge in edges["center"]:
                if edge.to_world_center:
                    continue

                line_segments.append(edge.line_set)

        draw_objects_for_demo(
            objects=[],
            points=points,
            lines=line_segments
        )

    def draw_top_graph(self):
        points = []
        line_segments = []

        for vertex in self.vertices:
            if vertex["top"].is_origin:
                continue

            points.append(vertex["top"].get_vertex())

        for edges in self.edges:
            for edge in edges["top"]:
                if edge.to_world_center:
                    continue

                line_segments.append(edge.line_set)

        draw_objects_for_demo(
            objects=[],
            points=points,
            lines=line_segments
        )

    def draw_front_graph(self):
        points = []
        line_segments = []

        for vertex in self.vertices:
            if vertex["front"].is_origin:
                continue

            points.append(vertex["front"].get_vertex())

        for edges in self.edges:
            for edge in edges["front"]:
                if edge.to_world_center:
                    continue

                line_segments.append(edge.line_set)

        draw_objects_for_demo(
            objects=[],
            points=points,
            lines=line_segments
        )
