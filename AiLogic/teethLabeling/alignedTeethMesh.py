import copy
import heapq

import numpy as np
import vedo

from .easyMeshClass import Easy_Mesh


class AlignedTeethMesh:
    def __init__(self, input_path):
        self.vedo_mesh = vedo.io.load(input_path)
        self.center_of_mass = self.vedo_mesh.centerOfMass()

        self.easy_mesh = Easy_Mesh(filename=input_path)
        self.easy_mesh.cell_attributes['Label'] = np.zeros([self.easy_mesh.cell_ids.shape[0], 1])
        self.easy_mesh.get_cell_curvatures()

        self.points_str = np.asarray([' '.join(item) for item in self.easy_mesh.points.astype(str)])
        self.points_float = []
        self.points_float_2d = []
        for item in self.easy_mesh.points.astype(float):
            self.points_float.append(item)
            self.points_float_2d.append(item[:-1])

        self.points_float = np.asarray(self.points_float)
        self.points_float_2d = np.asarray(self.points_float_2d)

        self.point_ids = {}

    def initialize_mesh(self):
        # initializing the mesh dictionaries
        for (i, point) in enumerate(np.asarray(self.easy_mesh.points)):
            self.point_ids[i] = {
                'point': point,
                'polygon_id': -1,
                'is_boundary_point': 0,  # 1: inner boundary point, 2: outer boundary point

                # outer boundary information
                'outer_boundary_point': None,
                'outer_boundary_point_id': None,
                'outer_boundary_point_euclidean_distance': 1000,
                'outer_boundary_point_mesh_distance': 1000,
                'parent_id_to_outer_boundary': -1,

                # inner boundary information
                'inner_boundary_point': None,
                'inner_boundary_point_id': None,
                'inner_boundary_point_euclidean_distance': 1000,
                'inner_boundary_point_mesh_distance': 1000,
                'parent_id_to_inner_boundary': -1,

                'cell_ids': set(),
                'min_cell_curvature': 1000,
                'min_cell_curvature_id': None,

                'neighbours': set(),
            }

        # saving the neighbours of mesh cell ids
        for (cell_id, point_ids) in enumerate(self.easy_mesh.cell_ids):
            # computing the center of a cell, to be used while giving labels to the
            # unrecognized polygons with less than 20 (current threshold) boundary points
            center = np.zeros(3)
            for point_id in point_ids:
                center += np.asarray(self.point_ids[point_id]['point'])
                self.point_ids[point_id]['cell_ids'].add(cell_id)  # adding cell_id to corresponding set

                if self.point_ids[point_id]['min_cell_curvature'] > self.easy_mesh.cell_attributes['Curvature'][
                    cell_id]:
                    self.point_ids[point_id]['min_cell_curvature'] = self.easy_mesh.cell_attributes['Curvature'][
                        cell_id]
                    self.point_ids[point_id]['min_cell_curvature_id'] = cell_id

            self.point_ids[point_ids[0]]['neighbours'] = self.point_ids[point_ids[0]]['neighbours'].union(
                {point_ids[1], point_ids[2]})
            self.point_ids[point_ids[1]]['neighbours'] = self.point_ids[point_ids[1]]['neighbours'].union(
                {point_ids[2], point_ids[0]})
            self.point_ids[point_ids[2]]['neighbours'] = self.point_ids[point_ids[2]]['neighbours'].union(
                {point_ids[0], point_ids[1]})

        # saving the neighbours of mesh cell ids
        # for (point_id, mesh_point) in self.point_ids.items():
        #     for cell_id in mesh_point['cell_ids']:
        #         for neighbour_cell_id in mesh_point['cell_ids']:
        #             if cell_id == neighbour_cell_id:
        #                 continue
        #
        #             self.cell_neighbours[cell_id].add(neighbour_cell_id)
        #             self.cell_neighbours_copy[cell_id].add(neighbour_cell_id)

    def get_point(self, point_id):
        return self.point_ids[point_id]['point']

    def find_boundary_parent_ids(
            self,
            boundary_type: str,  # outer or inner
            pids,  # point ids of the starting list of either outer or inner boundary points
            start_index: int,
            end_index: int,
    ):
        boundary_point_value = 2 if boundary_type == 'outer' else 1
        while (end_index - start_index) > 0:
            for i in range(start_index, end_index):
                point_id = pids[i]
                mesh_point = self.point_ids[point_id]
                for neighbour_point_id in self.point_ids[point_id]['neighbours']:
                    if neighbour_point_id == mesh_point['parent_id_to_' + boundary_type + '_boundary']:
                        continue

                    neighbour_mesh_point = self.point_ids[neighbour_point_id]

                    if neighbour_mesh_point['is_boundary_point'] == boundary_point_value:
                        continue

                    parent_dist = np.linalg.norm(
                        np.asarray(neighbour_mesh_point['point']) - np.asarray(mesh_point['point'])
                    )

                    if parent_dist == 0:
                        continue

                    mesh_dist = parent_dist + mesh_point[boundary_type + '_boundary_point_mesh_distance']

                    euc_dist = np.linalg.norm(
                        np.asarray(neighbour_mesh_point['point']) - np.asarray(
                            mesh_point[boundary_type + '_boundary_point'])
                    )

                    if neighbour_mesh_point[boundary_type + '_boundary_point'] is not None and \
                            mesh_dist >= neighbour_mesh_point[boundary_type + '_boundary_point_mesh_distance']:
                        continue

                    neighbour_mesh_point[boundary_type + '_boundary_point'] = mesh_point[
                        boundary_type + '_boundary_point']
                    neighbour_mesh_point[boundary_type + '_boundary_point_id'] = mesh_point[
                        boundary_type + '_boundary_point_id']
                    neighbour_mesh_point[boundary_type + '_boundary_point_mesh_distance'] = mesh_dist
                    neighbour_mesh_point[boundary_type + '_boundary_point_euclidean_distance'] = euc_dist
                    neighbour_mesh_point['parent_id_to_' + boundary_type + '_boundary'] = point_id
                    pids.append(neighbour_point_id)

            start_index = end_index
            end_index = len(pids)

    def calculate_curvature_heuristic(self, point_id, sink_point_id):
        cell_id = self.point_ids[point_id]['min_cell_curvature_id']
        h_value = self.easy_mesh.cell_attributes['Curvature'][cell_id]
        g_value = self.calculate_distance(point_id, sink_point_id)

        return g_value + (h_value * 20)

    def calculate_2d_heuristic(self, point_id, sink_point_id):
        point = self.get_point(point_id)
        sink_point = self.get_point(sink_point_id)
        g_value = np.linalg.norm(point[:-1] - sink_point[:-1])
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
                np.linalg.norm(aligned_mesh_points - point, axis=1) + (1000 * self.points_float[:, 2] < elevation_value)
            )
        else:
            new_point_id = np.argmin(
                np.linalg.norm(aligned_mesh_points - point, axis=1) + (1000 * self.points_float[:, 2] > elevation_value)
            )

        return new_point_id

    def restrict_points_for_astar(
            self,
            pids
    ):
        if len(pids) == 0:
            return []

        visited_points = {}
        for point_id in pids:
            visited_points[point_id] = 1

        start_index = 0
        end_index = len(pids)
        for _ in range(2):
            if (end_index - start_index) == 0:
                break

            for i in range(start_index, end_index):
                point_id = pids[i]

                for neighbour_point_id in self.point_ids[point_id]['neighbours']:
                    if neighbour_point_id in visited_points:
                        continue

                    visited_points[neighbour_point_id] = 1
                    pids.append(neighbour_point_id)

            start_index = end_index
            end_index = len(pids)

        return pids

    def find_astar_curvature_path(
            self,
            source_point_id,
            sink_point_id,
            restrict_point_ids=None,
    ):
        if source_point_id == sink_point_id:
            return [source_point_id]

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

        if not found_sink:
            return []

        path = []
        point_id = sink_point_id
        for _ in range(5000):
            path.append(point_id)
            if point_id not in parents:
                break

            point_id = parents[point_id]
            if point_id == -1:
                break

        return path


        # if len(path) == 1:
        #     path = []
        #
        #     geodesic_path_mesh = geodesic(
        #         vmesh=self.vedo_mesh,
        #         start=self.get_point(source_point_id),
        #         end=self.get_point(sink_point_id),
        #     )
        #
        #     for point in geodesic_path_mesh.points():
        #         point_id = np.where(self.points_str == ' '.join(point.astype(str)))[0][0]
        #         path.append(point_id)
        #
        #     path = path[::-1]
        #     if len(path) == 0:
        #         path = [sink_point_id]
