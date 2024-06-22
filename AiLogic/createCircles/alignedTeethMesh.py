from cmath import sin
import copy
import heapq

import numpy as np


class AlignedTeethMesh:
    def __init__(self, point_ids):
        self.point_ids = point_ids
        self.points_float = np.asarray([item['point'] for (_, item) in self.point_ids.items()])

    def get_point(self, point_id):
        return self.point_ids[point_id]['point']

    def calculate_curvature_heuristic(self, point_id, sink_point_id):
        h_value = self.point_ids[point_id]['min_cell_curvature']
        g_value = self.calculate_distance(point_id, sink_point_id)

        return g_value + (h_value * 5)

    def calculate_distance(self, point_id1, point_id2):
        return np.linalg.norm(self.get_point(point_id1) - self.get_point(point_id2))

    def find_closest_point(self, point):
        aligned_mesh_points = copy.deepcopy(self.points_float)
        new_point_id = np.argmin(np.linalg.norm(aligned_mesh_points - point, axis=1))
        return new_point_id

    def find_astar_curvature_path(
            self,
            source_point_id,
            sink_point_id,
            restrict_point_ids=None,
    ):
        if restrict_point_ids is None:
            restrict_point_ids = []

        source_value = self.calculate_curvature_heuristic(
            point_id=source_point_id,
            sink_point_id=sink_point_id
        )

        open_list = [(source_value, source_point_id)]
        closed_list = []

        parents = {source_point_id: -1}
        heapq.heapify(open_list)
        for _ in range(5000):
            (cur_value, cur_point_id) = heapq.heappop(open_list)
            closed_list.append((cur_value, cur_point_id))

            found_sink = False
            for neighbour_point_id in self.point_ids[cur_point_id]['neighbours']:
                if neighbour_point_id == source_point_id or \
                        neighbour_point_id in restrict_point_ids:
                    continue

                if neighbour_point_id == sink_point_id:
                    parents[sink_point_id] = cur_point_id
                    found_sink = True
                    break

                value = self.calculate_curvature_heuristic(
                    point_id=neighbour_point_id,
                    sink_point_id=sink_point_id
                )

                open_search_index = None
                for i in range(len(open_list)):
                    if open_list[i][1] == neighbour_point_id:
                        open_search_index = i
                        break

                closed_search_index = None
                for i in range(len(closed_list)):
                    if closed_list[i][1] == neighbour_point_id:
                        closed_search_index = i
                        break

                if open_search_index is not None:
                    if value >= open_list[open_search_index][0]:
                        continue

                    open_list.pop(open_search_index)

                elif closed_search_index is not None:
                    if value >= closed_list[closed_search_index][0]:
                        continue

                    closed_list.pop(closed_search_index)

                heapq.heappush(open_list, (value, neighbour_point_id))
                parents[neighbour_point_id] = cur_point_id

            if found_sink:
                break

            if len(open_list) == 0:
                break

        path = []
        point_id = sink_point_id
        for _ in range(1000):
            path.append(point_id)
            if point_id not in parents:
                break

            point_id = parents[point_id]
            if point_id == -1:
                break

        return path