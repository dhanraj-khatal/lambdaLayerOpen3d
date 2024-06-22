import copy
import heapq

import numpy as np
import vedo


class ToothEasyMesh:
    def __init__(self, input_path):
        self.vedo_mesh = vedo.load(input_path)

        self.points_float = []
        self.points_float_2d = []
        for item in self.vedo_mesh.points().astype(float):
            self.points_float.append(item)
            self.points_float_2d.append(item[:-1])

        self.points_float = np.asarray(self.points_float)
        self.points_float_2d = np.asarray(self.points_float_2d)

        self.point_ids = {}

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            if k == 'vedo_mesh':
                setattr(result, k, self.vedo_mesh.clone(deep=True))
                continue

            setattr(result, k, copy.deepcopy(v, memo))

        return result

    def get_point(self, point_id):
        return self.points_float[point_id]

    def find_closest_point(self, point):
        # return self.vedo_mesh.closest_point(point, return_point_id=True)
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
                np.linalg.norm(aligned_mesh_points - point, axis=1) + (1000 * self.points_float[:, 2] < elevation_value)
            )
        else:
            new_point_id = np.argmin(
                np.linalg.norm(aligned_mesh_points - point, axis=1) + (1000 * self.points_float[:, 2] > elevation_value)
            )

        return new_point_id

    def get_number_of_points(self):
        return len(self.points_float)

    def calculate_distance(self, point_id1, point_id2):
        return np.linalg.norm(self.get_point(point_id1) - self.get_point(point_id2))

    def calculate_heuristic(self, source_point_id, point_id, sink_point_id):
        g_value = self.calculate_distance(source_point_id, point_id)
        h_value = self.calculate_distance(point_id, sink_point_id)

        return h_value

    def find_astar_path(
            self,
            source_point_id,
            sink_point_id,
            restrict_point_ids=None,
    ):
        if source_point_id == sink_point_id:
            return [source_point_id]

        if restrict_point_ids is None:
            restrict_point_ids = []

        source_value = self.calculate_heuristic(
            source_point_id=source_point_id,
            point_id=source_point_id,
            sink_point_id=sink_point_id
        )

        open_list = [(source_value, source_point_id)]
        closed_list = []

        parents = {source_point_id: -1}
        heapq.heapify(open_list)
        found_sink = False
        for _ in range(10000):
            (cur_value, cur_point_id) = heapq.heappop(open_list)
            closed_list.append((cur_value, cur_point_id))

            for neighbour_point_id in self.point_ids[cur_point_id]['neighbours']:
                if neighbour_point_id == source_point_id or \
                        neighbour_point_id in restrict_point_ids:
                    continue

                if neighbour_point_id == sink_point_id:
                    parents[sink_point_id] = cur_point_id
                    found_sink = True
                    break

                value = self.calculate_heuristic(
                    source_point_id=source_point_id,
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

        if not found_sink:
            return []

        path = []
        points_path = []
        point_id = sink_point_id
        for _ in range(5000):
            path.append(point_id)
            points_path.append(self.get_point(point_id))
            if point_id not in parents:
                break

            point_id = parents[point_id]
            if point_id == -1:
                break

        return points_path

    def find_geodesic_path(
            self,
            source_point_id,
            sink_point_id,
    ):
        geodesic_path_mesh = self.vedo_mesh.geodesic(
            start=source_point_id,
            end=sink_point_id,
        )

        path = []
        for point in geodesic_path_mesh.points():
            path.append(point)
            # point_id = np.where(self.points_str == ' '.join(point.astype(str)))[0][0]

        # path = path[::-1]

        return path

