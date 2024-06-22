import copy
import heapq
from typing import List

import numpy as np

from AiLogic.teethLabeling.utils import get_slope


class AlignedTeethMesh:
    def __init__(self, point_ids):
        self.point_ids = point_ids

        self.points_float = []
        self.points_float_2d = []
        for (_, item) in self.point_ids.items():
            self.points_float.append(item['point'])
            self.points_float_2d.append(item['point'][:-1])

        self.points_float = np.asarray(self.points_float)
        self.points_float_2d = np.asarray(self.points_float_2d)
        # self.points_float = np.asarray([item['point'] for (_, item) in self.point_ids.items()])

    def get_point(self, point_id):
        return self.point_ids[point_id]['point']

    def calculate_curvature_heuristic(self, point_id, sink_point_id):
        h_value = self.point_ids[point_id]['min_cell_curvature']
        g_value = self.calculate_distance(point_id, sink_point_id)

        return g_value + (h_value * 20)

    def calculate_negative_curvature_heuristic(self, point_id, sink_point_id):
        h_value = self.point_ids[point_id]['min_cell_curvature']
        g_value = self.calculate_distance(point_id, sink_point_id)

        return g_value * 5 - (h_value * 5)

    def calculate_heuristic(self, point_id, sink_point_id, is_2d=True):
        point = self.get_point(point_id)
        sink_point = self.get_point(sink_point_id)
        if is_2d:
            g_value = np.linalg.norm(point[:-1] - sink_point[:-1])
        else:
            g_value = np.linalg.norm(point - sink_point)
        return g_value

    def calculate_distance(self, point_id1, point_id2):
        return np.linalg.norm(self.get_point(point_id1) - self.get_point(point_id2))

    def find_closest_point(self, point):
        aligned_mesh_points = copy.deepcopy(self.points_float)
        new_point_id = np.argmin(np.linalg.norm(aligned_mesh_points - point, axis=1))
        return new_point_id

    def find_closest_point_2d(
            self,
            point,
            elevation_value,
            check_min_z_value: bool = False
    ):
        aligned_mesh_points = copy.deepcopy(self.points_float_2d)
        if check_min_z_value:
            new_point_id = np.argmin(
                np.linalg.norm(aligned_mesh_points - point[:-1], axis=1) + (self.points_float[:, 2] > point[2])
            )
        else:
            new_point_id = np.argmin(
                np.linalg.norm(aligned_mesh_points - point[:-1], axis=1) + (self.points_float[:, 2] < point[2])
            )

        return new_point_id

    def find_astar_path_2d(
            self,
            source_point_id,
            sink_point_id,
            restrict_point_ids=None,
            is_2d=True,
    ):
        if restrict_point_ids is None:
            restrict_point_ids = []

        source_value = self.calculate_heuristic(
            point_id=source_point_id,
            sink_point_id=sink_point_id,
            is_2d=is_2d
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
                if neighbour_point_id == sink_point_id:
                    parents[sink_point_id] = cur_point_id
                    found_sink = True
                    break

                if neighbour_point_id == source_point_id:
                    continue

                if neighbour_point_id in restrict_point_ids:
                    continue

                value = self.calculate_heuristic(
                    point_id=neighbour_point_id,
                    sink_point_id=sink_point_id,
                    is_2d=is_2d
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

    def find_astar_path_with_negative_curvature(
            self,
            source_point_id,
            sink_point_id,
            restrict_point_ids=None,
    ):
        if restrict_point_ids is None:
            restrict_point_ids = []

        source_value = self.calculate_negative_curvature_heuristic(
            point_id=source_point_id,
            sink_point_id=sink_point_id,
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
                if neighbour_point_id == sink_point_id:
                    parents[sink_point_id] = cur_point_id
                    found_sink = True
                    break

                if neighbour_point_id == source_point_id:
                    continue

                if neighbour_point_id in restrict_point_ids:
                    continue

                value = self.calculate_negative_curvature_heuristic(
                    point_id=neighbour_point_id,
                    sink_point_id=sink_point_id,
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

    def find_astar_path_2d_with_boundary_check(
        self,
        source_point_id,
        sink_point_id,
        restrict_point_ids=None,
        boundary_point_ids=None,
        is_2d=True,
    ):
        if restrict_point_ids is None:
            restrict_point_ids = []

        if boundary_point_ids is None:
            boundary_point_ids = []

        source_value = self.calculate_heuristic(
            point_id=source_point_id,
            sink_point_id=sink_point_id,
            is_2d=is_2d
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
                if neighbour_point_id in boundary_point_ids:
                    sink_point_id = neighbour_point_id
                    parents[sink_point_id] = cur_point_id
                    found_sink = True
                    break

                if neighbour_point_id == sink_point_id:
                    parents[sink_point_id] = cur_point_id
                    found_sink = True
                    break

                if neighbour_point_id == source_point_id:
                    continue

                if neighbour_point_id in restrict_point_ids:
                    continue

                value = self.calculate_heuristic(
                    point_id=neighbour_point_id,
                    sink_point_id=sink_point_id,
                    is_2d=is_2d
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

    def find_straight_path(
            self,
            source_point_id: int,
            sink_point_id: int,
            no_of_points: int = 20,
    ):
        source_point = self.get_point(source_point_id)
        sink_point = self.get_point(sink_point_id)
        total_length = np.linalg.norm(source_point - sink_point)
        step_size = total_length / no_of_points

        slope = get_slope(points=[source_point, sink_point])

        path_points = []
        current_point = source_point + step_size * slope
        for i in range(no_of_points):
            path_points.append(self.find_closest_point(current_point))
            current_point = current_point + step_size * slope

        return path_points

