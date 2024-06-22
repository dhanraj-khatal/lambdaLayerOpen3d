import copy
import errno
import json
import logging
import os
import pickle
from typing import List

import pymeshlab
import time
import traceback
import open3d as o3d
from joblib import Parallel, delayed
from collections import deque

import numpy as np

from AiLogic.createCircles.utils import save_as_json
from AiLogic.teethLabeling.alignedTeethMesh import AlignedTeethMesh
from AiLogic.teethLabeling.easyMeshClass import Easy_Mesh
from AiLogic.teethLabeling.utils import MeshConv, obj_to_stl, get_json_object, save_stl_from_easy_mesh, get_slope

logger = logging.getLogger()


def initialize_mesh(
        easy_mesh: Easy_Mesh,
        get_cell_neighbours: bool = False
):
    mesh_points = easy_mesh.points
    mesh_point_ids = {}
    mesh_cell_ids = {}

    # initializing the easy_mesh dictionaries
    for (i, point) in enumerate(np.asarray(mesh_points)):
        mesh_point_ids[i] = {
            'point': point,
            'polygon_id': -1,
            'cell_ids': set(),
            'neighbours': set(),
        }

    # saving the neighbours of easy_mesh point ids
    for (cell_id, cell_point_ids) in enumerate(easy_mesh.cell_ids):
        barry_center = np.asarray((0, 0, 0)),

        for point_id in cell_point_ids:
            barry_center = barry_center + mesh_point_ids[point_id]['point']
            mesh_point_ids[point_id]['cell_ids'].add(cell_id)  # adding cell_id to corresponding set

        point1 = mesh_point_ids[cell_point_ids[0]]['point']
        point2 = mesh_point_ids[cell_point_ids[1]]['point']
        point3 = mesh_point_ids[cell_point_ids[2]]['point']

        vector1 = get_slope(points=[point1, point2])
        vector2 = get_slope(points=[point1, point3])

        cell_normal = np.cross(vector1, vector2)
        cell_normal_length = np.linalg.norm(cell_normal)
        cell_normal /= cell_normal_length

        mesh_cell_ids[cell_id] = {
            'neighbours': set(),
            'barry_center': barry_center[0] / 3,
            'normal_vector': cell_normal
        }

        mesh_point_ids[cell_point_ids[0]]['neighbours'] = mesh_point_ids[cell_point_ids[0]]['neighbours'].union(
            {cell_point_ids[1], cell_point_ids[2]}
        )

        mesh_point_ids[cell_point_ids[1]]['neighbours'] = mesh_point_ids[cell_point_ids[1]]['neighbours'].union(
            {cell_point_ids[0], cell_point_ids[2]}
        )

        mesh_point_ids[cell_point_ids[2]]['neighbours'] = mesh_point_ids[cell_point_ids[2]]['neighbours'].union(
            {cell_point_ids[1], cell_point_ids[0]}
        )

    if get_cell_neighbours:
        # saving the neighbours of easy_mesh_without_gums cell ids
        for (point_id, mesh_point) in mesh_point_ids.items():
            for cell_id in mesh_point['cell_ids']:
                for neighbour_cell_id in mesh_point['cell_ids']:
                    if cell_id == neighbour_cell_id:
                        continue

                    mesh_cell_ids[cell_id]['neighbours'].add(neighbour_cell_id)

    return mesh_point_ids, mesh_cell_ids


def give_label_to_circles(
        circles_folder: str,
        circle_filename: str,
        mesh_with_gums_pkl: AlignedTeethMesh,
        easy_mesh_with_gums: Easy_Mesh,
        tooth_circle_points_ids: List,
        tooth_circle_points: List,
):
    circle_path = os.path.join(circles_folder, circle_filename)
    if not os.path.isfile(circle_path):
        return

    circle_json_points = get_json_object(circle_path)

    for point in circle_json_points:
        point_id = mesh_with_gums_pkl.find_closest_point(point)
        if point_id not in mesh_with_gums_pkl.point_ids:
            continue

        tooth_circle_points.append(np.asarray(point))
        tooth_circle_points_ids.append(point_id)

        for cell_id in mesh_with_gums_pkl.point_ids[point_id]['cell_ids']:
            easy_mesh_with_gums.cell_attributes['Label'][cell_id] = 1


def give_cell_labels_to_path(
        mesh_with_gums_pkl: AlignedTeethMesh,
        easy_mesh_with_gums: Easy_Mesh,
        source_point_id: int,
        sink_point_id: int,
        circle_point_ids: List,
        tooth_points: List,
        tooth_label: int,
):
    path = mesh_with_gums_pkl.find_astar_path_2d(
        source_point_id=source_point_id,
        sink_point_id=sink_point_id,
        restrict_point_ids=circle_point_ids,
        is_2d=False
    )

    for point_id in path:
        tooth_points.append(mesh_with_gums_pkl.get_point(point_id))

        for cell_id in mesh_with_gums_pkl.point_ids[point_id]['cell_ids']:
            if easy_mesh_with_gums.cell_attributes['Label'][cell_id] != -1:
                easy_mesh_with_gums.cell_attributes['Label'][cell_id] = tooth_label


def save_clean_mesh(input_filename, output_filename):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(input_filename)
    ms.compute_selection_by_non_manifold_edges_per_face()
    ms.meshing_remove_selected_vertices_and_faces()
    ms.compute_selection_by_non_manifold_per_vertex()
    ms.meshing_remove_selected_vertices()
    ms.meshing_remove_connected_component_by_face_number(mincomponentsize=50, removeunref=1)

    ms.save_current_mesh(output_filename)


def save_segmented_teeth(label, temp, out, vtp_object):
    if label == 0:
        return
        # obj_file_name = os.path.join(temp, case_type + "_gums.obj")
        # stl_file_name = os.path.join(out, case_type + "_gums.stl")
    else:
        case_tooth_name = "Tooth_" + str(int(label))
        obj_file_name = os.path.join(temp, case_tooth_name + ".obj")
        stl_file_name = os.path.join(out, case_tooth_name + ".stl")

    vtp_object.to_obj_label(obj_file_name, label)

    obj_to_stl(obj_file_name, stl_file_name)


def find_closest_point(mesh_points_str, point):
    return np.where(mesh_points_str == ' '.join(point.astype(str)))[0][0]


def stitch_together_open_boundary_points(
        boundary_points_ordered_per_polygon: dict
):
    for aaa in range(30):
        open_polygons = []
        for (polygon_id, boundary_points) in boundary_points_ordered_per_polygon.items():
            euc_dist = np.linalg.norm(boundary_points[0] - boundary_points[-1])
            if euc_dist < POLYGON_POINT_DISTANCE_THRESHOLD:
                continue

            open_polygons.append(polygon_id)

        if len(open_polygons) == 0:
            break

        visited_polygons = {}
        for polygon_id1 in open_polygons:
            # below condition is required because after the connection of 2 polygons,
            # the 2nd polygon is deleted from the polygons list
            if polygon_id1 not in boundary_points_ordered_per_polygon:
                continue

            if polygon_id1 in visited_polygons:
                continue

            boundary_points1 = boundary_points_ordered_per_polygon[polygon_id1]
            point11 = boundary_points1[0]
            point12 = boundary_points1[-1]

            for polygon_id2 in open_polygons:
                if polygon_id2 not in boundary_points_ordered_per_polygon:
                    continue

                if polygon_id1 == polygon_id2:
                    continue

                boundary_points2 = boundary_points_ordered_per_polygon[polygon_id2]
                point21 = boundary_points2[0]
                point22 = boundary_points2[-1]

                dist11 = np.linalg.norm(point11 - point21)
                dist12 = np.linalg.norm(point11 - point22)

                dist21 = np.linalg.norm(point12 - point21)
                dist22 = np.linalg.norm(point12 - point22)

                if dist11 < POLYGON_POINT_DISTANCE_THRESHOLD:
                    visited_polygons[polygon_id1] = polygon_id2
                    visited_polygons[polygon_id2] = polygon_id1
                    new_boundary_points = np.concatenate([boundary_points1[::-1], boundary_points2])

                elif dist22 < POLYGON_POINT_DISTANCE_THRESHOLD:
                    visited_polygons[polygon_id1] = polygon_id2
                    visited_polygons[polygon_id2] = polygon_id1
                    new_boundary_points = np.concatenate([boundary_points1, boundary_points2[::-1]])

                elif dist12 < POLYGON_POINT_DISTANCE_THRESHOLD:
                    visited_polygons[polygon_id1] = polygon_id2
                    visited_polygons[polygon_id2] = polygon_id1
                    new_boundary_points = np.concatenate([boundary_points2, boundary_points1])

                elif dist21 < POLYGON_POINT_DISTANCE_THRESHOLD:
                    visited_polygons[polygon_id1] = polygon_id2
                    visited_polygons[polygon_id2] = polygon_id1
                    new_boundary_points = np.concatenate([boundary_points1, boundary_points2])

                else:
                    continue

                boundary_points_ordered_per_polygon[polygon_id1] = new_boundary_points

                del boundary_points_ordered_per_polygon[polygon_id2]
                break


def find_leftmost_and_rightmost_indices(
        points_list: np.array,
        center_point: np.array,
        x_max1: float = -10000,
        x_max2: float = -10000,
):
    """
    Find the leftmost and rightmost points in a points list based on
    x-coordinates of the points wrt the center point.

    Leftmost and rightmost indices are defined on the aligned mesh which
    is bisected by +ve x-axis and y-axis as the tangent at the root.

    @param x_max1: default: -10000
    @param x_max2: default: -10000
    @param points_list: a list of points defining a boundary
    @param center_point: a point defining the root/center of the parabola
    @return: indices of leftmost and rightmost points in the points_list
    """
    leftmost_i = None
    rightmost_i = None

    for (i, point) in enumerate(points_list):
        if point[1] > center_point[1]:
            if point[0] > x_max1:
                x_max1 = point[0]
                leftmost_i = i

        else:
            if point[0] > x_max2:
                x_max2 = point[0]
                rightmost_i = i

    return leftmost_i, rightmost_i


def find_closest_points_in_2_boundaries(
        points1: np.array,
        points2: np.array,
):
    """
    Find the indices of the closest points in 2 point list

    @param points1: 1st points list
    @param points2: 2nd points list
    @return:
    """
    min_dist = 1000
    nearest_point1_id = None
    nearest_point2_id = None

    for (i1, point1) in enumerate(points1):
        i2 = np.argmin(np.linalg.norm(points2 - point1, axis=1))
        dist = np.linalg.norm(point1 - points2[i2])
        if dist < min_dist:
            min_dist = dist
            nearest_point1_id = i1
            nearest_point2_id = i2

    return min_dist, nearest_point1_id, nearest_point2_id


def tooth_fill_hole(
        filepath: str,
        output_filepath: str
):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(filepath)
    ms.meshing_close_holes(maxholesize=10, selfintersection=0)
    ms.meshing_surface_subdivision_ls3_loop(threshold=pymeshlab.Percentage(0.100))
    ms.save_current_mesh(output_filepath)


def teeth_separator_labeling(
        jaw_type,
        input_aligned_mesh_stl_file,
        input_vtp_file,
        labels_json,
        out,
        temp,
        circles_folder,
        mesh_without_gums_clean_stl_path
):
    """
    Teeth labelling

    @param jaw_type: mand or max
    @param input_aligned_mesh_stl_file: Path for the STL file of the aligned mesh
    @param input_vtp_file: Path for the VTP file of the aligned mesh
    @param labels_json: Path for the JSON file of the not marked missing teeth
    @param out: Output folder path
    @param temp: Temp folder path
    @param circles_folder: Circles folder path containing the circle json files
    @return:
    :param mesh_without_gums_clean_stl_path: sdfsd
    """
    try:
        # logger.info(f'Out: {out}')
        # logger.info(f'temp: {temp}')
        # logger.info(f'circles_folder: {circles_folder}')
        # logger.info({os.path.isdir(circles_folder)})

        start_time = time.time()

        if not os.path.isfile(input_vtp_file):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), input_vtp_file)

        # create temp and out folders
        os.makedirs(temp, exist_ok=True)
        os.makedirs(out, exist_ok=True)

        # os.mkdir(temp)
        # os.mkdir(out)

        # loading teeth labels json file obtained from Mark Missing Tool
        json_file = open(labels_json)
        data = json.load(json_file)
        json_file.close()

        if jaw_type == "max":
            vtp_output_path = os.path.join(out, "maxillary_teeth.vtp")
            case_type = "maxillary"
            teeth_labels = data[case_type]
        else:
            vtp_output_path = os.path.join(out, "mandibular_teeth.vtp")
            case_type = "mandibular"
            teeth_labels = data[case_type][::-1]

        st = time.time()

        # # logger.info({os.path.isdir(circles_folder)})
        # if len(os.listdir(circles_folder)) != len(teeth_labels):
        #     logger.error('Error: No of circles are not same as the no of teeth marked not missing in teeth_labels.json')
        #     # return 'Error: No of circles are not same as the no of teeth marked not missing in teeth_labels.json'
        #     return 'CIRCLES_TEETH_LABLES_DOES_NOT_MATCH'
        #
        # logger.info(f'Step 1: Teeth labels for this case: {" ".join(np.asarray(teeth_labels).astype(str))}')
        #
        # # starting the timer to calculate the time taken for teeth segmentation per case
        # # f = open(input_pkl_file, 'rb')
        # # point_ids = pickle.load(f)
        # # mesh_with_gums_pkl = AlignedTeethMesh(point_ids)
        # mesh_with_gums_pkl = AlignedTeethMesh(input_aligned_mesh_stl_file)
        # logger.info(f'{time.time()}: Step 1.1: load mesh with gums')
        #
        # easy_mesh_with_gums = Easy_Mesh(filename=input_vtp_file)
        # easy_mesh_with_gums.cell_attributes['Label'] = np.zeros([easy_mesh_with_gums.cell_ids.shape[0], 1])
        # mesh_with_gums_boundary_points = easy_mesh_with_gums.get_ordered_boundary_points()

        o3d_mesh_with_gums = o3d.io.read_triangle_mesh(input_aligned_mesh_stl_file)
        o3d_mesh_with_gums.compute_vertex_normals()
        o3d_mesh_with_gums_center = o3d_mesh_with_gums.get_center()

        #
        # objects = []
        # for circle_filename in os.listdir(circles_folder):
        #     circle_path = os.path.join(circles_folder, circle_filename)
        #     if not os.path.isfile(circle_path):
        #         continue
        #
        #     circle_points = np.asarray(get_json_object(circle_path))
        #
        #     pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(circle_points))
        #     pcd.paint_uniform_color([1, 0, 0])
        #
        #     objects.append(pcd)
        #
        # o3d.visualization.draw_geometries([o3d_mesh_with_gums, *objects])
        #
        # tooth_circle_points_ids = []
        # tooth_circle_points = []
        # Parallel(n_jobs=4, require='sharedmem')(
        #     delayed(give_label_to_circles)(
        #         circles_folder,
        #         circle_filename,
        #         mesh_with_gums_pkl,
        #         easy_mesh_with_gums,
        #         tooth_circle_points_ids,
        #         tooth_circle_points,
        #     )
        #     for circle_filename in os.listdir(circles_folder)
        # )
        #
        # logger.info(f'{time.time()}: Step 1.2: read the input vtp file {input_vtp_file} with '
        #             f'{len(easy_mesh_with_gums.points)} points')
        #
        # bigger_polygon_id_length = 0
        # bigger_polygon_id = 1
        # polygon_count = 0
        #
        # objects = []
        # for (polygon_id, boundary_points) in mesh_with_gums_boundary_points.items():
        #     polygon_count += 1
        #
        #     if len(boundary_points) > bigger_polygon_id_length:
        #         bigger_polygon_id_length = len(boundary_points)
        #         bigger_polygon_id = polygon_id
        #
        #     pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(boundary_points))
        #
        #     objects.append(pcd)
        #
        # o3d.visualization.draw_geometries([o3d_mesh_with_gums, *objects])
        #
        # if polygon_count > 1:
        #     for (polygon_id, boundary_points) in mesh_with_gums_boundary_points.items():
        #         if polygon_id == bigger_polygon_id:
        #             continue
        #
        #         for point in boundary_points:
        #             point_id = mesh_with_gums_pkl.find_closest_point(point)
        #             for cell_id in mesh_with_gums_pkl.point_ids[point_id]['cell_ids']:
        #                 easy_mesh_with_gums.cell_attributes['Label'][cell_id] = 1
        #
        #     logger.warning(f'Polygon count of aligned mesh is greater than 1: {polygon_count}')
        #
        # bigger_boundary_points_list = mesh_with_gums_boundary_points[bigger_polygon_id]
        # logger.info(f'{np.shape(bigger_boundary_points_list)=}')
        #
        # avg_boundary_center_point = np.mean(bigger_boundary_points_list, axis=0)
        #
        # logger.info(f'Step 1.2.1: found {np.shape(avg_boundary_center_point)=}')
        #
        # leftmost_i, rightmost_i = find_leftmost_and_rightmost_indices(
        #     points_list=bigger_boundary_points_list,
        #     center_point=avg_boundary_center_point
        # )
        #
        # logger.info(f'Step 1.3: found {leftmost_i=} and {rightmost_i=} points on the bigger polygon')
        #
        # if leftmost_i > rightmost_i:
        #     count = (leftmost_i - rightmost_i + 1) // 2
        #     middle_i = rightmost_i + count
        # else:
        #     count = (rightmost_i - leftmost_i + 1) // 2
        #     middle_i = leftmost_i + count
        #
        # middle_point_near_incisors = bigger_boundary_points_list[middle_i]
        # middle_point_near_incisors_point_id = mesh_with_gums_pkl.find_closest_point(middle_point_near_incisors)
        # for cell_id in mesh_with_gums_pkl.point_ids[middle_point_near_incisors_point_id]['cell_ids']:
        #     easy_mesh_with_gums.cell_attributes['Label'][cell_id] = 1
        #
        # q = deque()
        # q.append(middle_point_near_incisors_point_id)
        # visited_points = {middle_point_near_incisors_point_id: 1}
        #
        # while len(q) > 0:
        #     front_point_id = q.popleft()
        #
        #     neighbours = mesh_with_gums_pkl.point_ids[front_point_id]['neighbours']
        #     for neighbour_point_id in neighbours:
        #         if neighbour_point_id in visited_points:
        #             continue
        #
        #         visited_points[neighbour_point_id] = 1
        #         if neighbour_point_id in tooth_circle_points_ids:
        #             continue
        #
        #         for cell_id in mesh_with_gums_pkl.point_ids[neighbour_point_id]['cell_ids']:
        #             easy_mesh_with_gums.cell_attributes['Label'][cell_id] = 1
        #
        #         q.append(neighbour_point_id)
        #
        # logger.info('Step 1.4: give labels to all the gum cells')
        #
        # logger.info(f'{time.time()}: Step 1.3: labels given to gums cells and all '
        #             f'the circles in the circles folder: {circles_folder}')

        # mesh_without_gums_clean_stl_path = save_stl_from_easy_mesh(
        #     easy_mesh_with_gums,
        #     temp,
        #     out,
        #     cell_label=0,
        #     filename_without_extension='aligned_mesh_without_gums',
        #     to_clean_mesh=True,
        #     to_log=True,
        #     save_in_output=True,
        # )

        logger.info(f'Step 1.5: Deleted the circle triangles in time {time.time() - st}')

        easy_mesh_without_gums: Easy_Mesh = Easy_Mesh(filename=mesh_without_gums_clean_stl_path)
        o3d_mesh_without_gums = o3d.io.read_triangle_mesh(mesh_without_gums_clean_stl_path)
        o3d_mesh_without_gums.compute_vertex_normals()

        mesh_cell_polygon_id = np.zeros(easy_mesh_without_gums.cell_ids.shape[0]).astype(int)
        easy_mesh_without_gums.cell_attributes['Label'] = np.zeros([easy_mesh_without_gums.cell_ids.shape[0], 1])

        st = time.time()
        easy_mesh_without_gums_points = easy_mesh_without_gums.points
        mesh_without_gums_points_str = np.asarray(
            [' '.join(item) for item in easy_mesh_without_gums_points.astype(str)])
        mesh_without_gums_point_ids, mesh_without_gums_cell_ids = initialize_mesh(
            easy_mesh=easy_mesh_without_gums,
            get_cell_neighbours=True
        )
        logger.info(f'Step 2: Initialization of mesh without gums complete in time {time.time() - st}')

        circles_count = 0
        tooth_circles = {}
        for circle_filename in os.listdir(circles_folder):
            circles_count += 1
            circle_path = os.path.join(circles_folder, circle_filename)
            # logger.info(f'{circle_path} Is File: {os.path.isfile(circle_path)}')
            # if (os.path.isfile(circle_path)):
            circle_points = np.asarray(get_json_object(circle_path))

            # circle_point_ids_with_gums = []
            # for point in circle_points:
            #     circle_point_ids_with_gums.append(mesh_with_gums_pkl.find_closest_point(point))

            circle_center = np.mean(circle_points, axis=0)

            tooth_circles[circles_count] = {
                'center': circle_center,
                'points': copy.deepcopy(circle_points),
                # 'point_ids': copy.deepcopy(circle_point_ids_with_gums),
            }

        logger.info(f'Step 3: Save tooth circles information')

        # finding the leftmost and rightmost points
        leftmost_point = {
            'circle_id': -1,
            'boundary_point_id_in_circle': -1,
            'point_id': -1,
            'point': [0, 0, 0]
        }
        rightmost_point = {
            'circle_id': -1,
            'boundary_point_id_in_circle': -1,
            'point_id': -1,
            'point': [0, 0, 0]
        }

        x_max1 = 0
        x_max2 = 0

        for (circle_id, circle) in tooth_circles.items():
            boundary_points = circle['points']

            leftmost_i, rightmost_i = find_leftmost_and_rightmost_indices(
                points_list=boundary_points,
                center_point=o3d_mesh_with_gums_center,
                x_max1=x_max1,
                x_max2=x_max2,
            )

            if leftmost_i is not None:
                x_max1 = boundary_points[leftmost_i][0]
                leftmost_point = {
                    'circle_id': circle_id,
                    'boundary_point_id_in_circle': leftmost_i,
                    'point_id': -1,
                    'point': np.asarray(boundary_points[leftmost_i])
                }

            if rightmost_i is not None:
                x_max2 = boundary_points[rightmost_i][0]
                rightmost_point = {
                    'circle_id': circle_id,
                    'boundary_point_id_in_circle': rightmost_i,
                    'point_id': -1,
                    'point': np.asarray(boundary_points[rightmost_i])
                }

            # c_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=o3d_mesh_with_gums_center)
            #
            # pcd_center = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([o3d_mesh_with_gums_center]))
            # pcd_center.paint_uniform_color([0, 1, 0])
            #
            # pcd_left = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([leftmost_point['point']]))
            # pcd_left.paint_uniform_color([1, 0, 0])
            #
            # pcd_right = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([rightmost_point['point']]))
            # pcd_right.paint_uniform_color([0, 0, 1])
            #
            # o3d.visualization.draw_geometries([o3d_mesh_with_gums, pcd_left, pcd_right, pcd_center, c_mesh])

        logger.info(f'Step 3: Save leftmost and rightmost indices for circle points')

        # finding the closest circles for each of the circle going from
        # leftmost point to rightmost point

        labelled_circles_folder_path = os.path.join(out, 'labelledCircles')
        os.mkdir(labelled_circles_folder_path)

        st = time.time()
        visited_circles = {}
        closest_circles = {}

        circle_id1 = leftmost_point['circle_id']
        circle_ids_from_left_to_right = []
        for i in range(len(tooth_circles.keys())):
            if circle_id1 == -1:
                break

            tooth_label = teeth_labels[i]
            circle_ids_from_left_to_right.append(circle_id1)

            save_as_json(
                data=tooth_circles[circle_id1]['points'],
                file_name=os.path.join(labelled_circles_folder_path, f'Tooth_{tooth_label}_circle.json')
            )

            boundary_points1 = np.asarray(tooth_circles[circle_id1]['points'])
            visited_circles[circle_id1] = {}

            closest_polygon = {
                'circle_id': -1,
                'distance': 10000,
                'point1_id': None,
                'point1': np.zeros(3),
                'point2_id': None,
                'point2': np.zeros(3),
            }

            for (circle_id2, circle2) in tooth_circles.items():
                if circle_id2 == circle_id1 or circle_id2 in visited_circles:
                    continue

                boundary_points2 = np.asarray(circle2['points'])

                visited_circles[circle_id1][circle_id2] = True

                min_dist, nearest_point1_id, nearest_point2_id = find_closest_points_in_2_boundaries(
                    points1=boundary_points1,
                    points2=boundary_points2
                )

                if min_dist < closest_polygon['distance']:
                    closest_polygon = {
                        'circle_id': circle_id2,
                        'distance': min_dist,
                        'point1_id': nearest_point1_id,
                        'point1': np.asarray(boundary_points1[nearest_point1_id]),
                        'point2': np.asarray(boundary_points2[nearest_point2_id]),
                        'point2_id': nearest_point2_id,
                    }

            closest_circles[circle_id1] = closest_polygon
            circle_id1 = closest_polygon['circle_id']

        # setting the closing point for the polygon of the rightmost point
        closest_circles[rightmost_point['circle_id']]['point1_id'] = rightmost_point['boundary_point_id_in_circle']

        logger.info(f'{circle_ids_from_left_to_right=}')
        logger.info(f'{teeth_labels=}')
        logger.info(f'Step 7: found the closest circles in time {time.time() - st}')

        # for (j, circle_id) in enumerate(circle_ids_from_left_to_right):
        #     circle = tooth_circles[circle_id]
        #     tooth_label = teeth_labels[j]
        #     print(f'********** {tooth_label=} {j=}')
        #     pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(circle['points']))
        #     pcd.paint_uniform_color([1, 0, 0])
        #     o3d.visualization.draw_geometries([o3d_mesh_with_gums, pcd])

        st = time.time()
        boundary_points_ordered_per_polygon = easy_mesh_without_gums.get_ordered_boundary_points()
        logger.info(f'Step 3: saved boundary_points_ordered_per_polygon '
                    f'{len(boundary_points_ordered_per_polygon.keys())} in time {time.time() - st}')

        # checking if the polygons are closed or not

        # looping over the list of polygons 30 times to determine the
        # open polygons and connecting if we found a pair of them
        st = time.time()
        stitch_together_open_boundary_points(boundary_points_ordered_per_polygon)
        logger.info(f'Step 4: checked if the polygons are closed or not in time {time.time() - st}')

        logger.info(f'Step 5: boundary polygon ids '
                    f'{" ".join(np.asarray(list(boundary_points_ordered_per_polygon.keys())).astype(str))}')

        # assigning polygon ids to each cell
        cell_count = {}
        duplicate_polygon_ids = {}
        for (polygon_id, boundary_points) in boundary_points_ordered_per_polygon.items():
            if polygon_id not in cell_count:
                cell_count[polygon_id] = 0
                duplicate_polygon_ids[polygon_id] = [polygon_id]

            start_point = boundary_points_ordered_per_polygon[polygon_id][0]
            start_point_id = find_closest_point(mesh_without_gums_points_str, point=start_point)
            start_cell_id = list(mesh_without_gums_point_ids[start_point_id]['cell_ids'])[0]

            start_polygon_id = int(mesh_cell_polygon_id[start_cell_id])
            if start_polygon_id != 0:
                duplicate_polygon_ids[start_polygon_id].append(polygon_id)
                duplicate_polygon_ids[polygon_id].append(start_polygon_id)
                continue

            mesh_cell_polygon_id[start_cell_id] = polygon_id
            cell_count[polygon_id] += 1

            q = deque()
            q.append(start_cell_id)
            visited_cells = {start_cell_id: 1}

            while len(q) > 0:
                front_cell_id = q.popleft()

                neighbours = mesh_without_gums_cell_ids[front_cell_id]['neighbours']
                for neighbour_cell_id in neighbours:

                    if neighbour_cell_id in visited_cells:
                        continue

                    visited_cells[neighbour_cell_id] = 1

                    mesh_cell_polygon_id[neighbour_cell_id] = polygon_id
                    cell_count[polygon_id] += 1

                    q.append(neighbour_cell_id)

        logger.info(f'{duplicate_polygon_ids=}')

        # finding the leftmost and rightmost points
        st = time.time()
        leftmost_point = {
            'polygon_id': -1,
            'boundary_point_id_in_polygon': -1,
            'point_id': -1,
            'point': [0, 0, 0]
        }

        rightmost_point = {
            'polygon_id': -1,
            'boundary_point_id_in_polygon': -1,
            'point_id': -1,
            'point': [0, 0, 0]
        }

        x_max1 = 0
        x_max2 = 0

        for (polygon_id, boundary_points) in boundary_points_ordered_per_polygon.items():
            leftmost_i, rightmost_i = find_leftmost_and_rightmost_indices(
                points_list=boundary_points,
                center_point=o3d_mesh_with_gums_center,
                x_max1=x_max1,
                x_max2=x_max2,
            )

            if leftmost_i is not None:
                x_max1 = boundary_points[leftmost_i][0]
                leftmost_point = {
                    'polygon_id': polygon_id,
                    'boundary_point_id_in_polygon': leftmost_i,
                    'point_id': -1,
                    'point': np.asarray(boundary_points[leftmost_i])
                }

            if rightmost_i is not None:
                x_max2 = boundary_points[rightmost_i][0]
                rightmost_point = {
                    'polygon_id': polygon_id,
                    'boundary_point_id_in_polygon': rightmost_i,
                    'point_id': -1,
                    'point': np.asarray(boundary_points[rightmost_i])
                }

        logger.info(f'Step 6: found the leftmost and rightmost points in the only teeth stl in time {time.time() - st}')

        # finding the closest polygons for each of the polygon going from
        # leftmost point to rightmost point
        st = time.time()
        visited_polygons = {}
        closest_polygons = {}

        polygon_id1 = leftmost_point['polygon_id']
        polygon_ids_from_left_to_right = []
        for _ in range(len(boundary_points_ordered_per_polygon.keys())):
            if polygon_id1 == -1:
                break

            polygon_ids_from_left_to_right.append(polygon_id1)

            boundary_points1 = boundary_points_ordered_per_polygon[polygon_id1]
            visited_polygons[polygon_id1] = {}

            closest_polygon = {
                'polygon_id': -1,
                'distance': 10000,
                'point1_id': None,
                'point1': np.zeros(3),
                'point2_id': None,
                'point2': np.zeros(3),
            }

            for (polygon_id2, boundary_points2) in boundary_points_ordered_per_polygon.items():
                if polygon_id2 == polygon_id1 or polygon_id2 in visited_polygons:
                    continue

                visited_polygons[polygon_id1][polygon_id2] = True

                min_dist, nearest_point1_id, nearest_point2_id = find_closest_points_in_2_boundaries(
                    points1=boundary_points1,
                    points2=boundary_points2
                )

                if min_dist < closest_polygon['distance']:
                    closest_polygon = {
                        'polygon_id': polygon_id2,
                        'distance': min_dist,
                        'point1_id': nearest_point1_id,
                        'point1': np.asarray(boundary_points1[nearest_point1_id]),
                        'point2': np.asarray(boundary_points2[nearest_point2_id]),
                        'point2_id': nearest_point2_id,
                    }

            closest_polygons[polygon_id1] = closest_polygon
            polygon_id1 = closest_polygon['polygon_id']

        logger.info(f'Step 7: found the closest polygons in time {time.time() - st}')

        # setting the closing point for the polygon of the rightmost point
        closest_polygons[rightmost_point['polygon_id']]['point1_id'] = rightmost_point['boundary_point_id_in_polygon']
        logger.info('giving teeth labels')
        polygon_labels = {}

        logger.info(f'Step 5: {polygon_ids_from_left_to_right=}')

        saved_tooth_labels = {tooth_label: False for tooth_label in teeth_labels}
        for (i, polygon_id) in enumerate(polygon_ids_from_left_to_right):
            if len(np.unique(np.asarray(duplicate_polygon_ids[polygon_id]))) > 1:
                logger.warning(f'Tooth circle for polygon {polygon_id} not saved as '
                               f'it is joined with either of its neighbour tooth!')
                continue

            boundary_points = np.asarray(boundary_points_ordered_per_polygon[polygon_id])
            polygon_center = np.mean(boundary_points, axis=0)

            tooth_label = -1
            closest_circle_id = -1
            closest_circle_center_distance = 10000
            for (j, circle_id) in enumerate(circle_ids_from_left_to_right):
                circle = tooth_circles[circle_id]
                distance = np.linalg.norm(polygon_center - circle['center'])
                if distance < closest_circle_center_distance:
                    closest_circle_center_distance = distance
                    closest_circle_id = circle_id
                    tooth_label = teeth_labels[j]

            if closest_circle_id == -1 or closest_circle_center_distance > 1 or tooth_label == -1:
                logger.warning(f'Tooth circle for polygon {polygon_id} with {tooth_label=} not saved!')
                continue

            logger.info(f'Tooth circle for {polygon_id=} with {tooth_label=} and {closest_circle_id=}')

            polygon_labels[polygon_id] = tooth_label
            saved_tooth_labels[tooth_label] = True

        logger.info(f'{polygon_labels=}')

        logger.info('Step 8: assigned label to each polygon id')

        st = time.time()
        for (cell_id, polygon_id) in enumerate(mesh_cell_polygon_id):
            if int(polygon_id) == 0 or polygon_id not in polygon_labels:
                continue

            easy_mesh_without_gums.cell_attributes['Label'][cell_id] = polygon_labels[polygon_id]

        logger.info(f'Step 9: Assigned label to all the cells in each polygon in time {time.time() - st}')

        st = time.time()
        easy_mesh_without_gums.to_vtp(vtp_output_path)
        vtp_object = MeshConv(vtp_output_path)

        for label in np.unique(vtp_object.cell_attributes['Label']):
            if label == 0:
                continue

            save_segmented_teeth(label, temp, out, vtp_object)

        logger.info('Checking if all teeth STL files have been created or not')

        not_save_teeth_folder_path = os.path.join(out, 'notSavedTeethCircles')
        os.mkdir(not_save_teeth_folder_path)

        teeth_not_saved = []
        # checking if the teeth are saved successfully or not
        for (i, tooth_label) in enumerate(teeth_labels):
            if saved_tooth_labels[tooth_label]:
                continue

            teeth_not_saved.append(tooth_label)
            saved_tooth_labels[tooth_label] = True

            circle_id = circle_ids_from_left_to_right[i]
            circle = tooth_circles[circle_id]
            circle_points = circle['points']

            save_as_json(
                circle_points,
                os.path.join(not_save_teeth_folder_path, f'Tooth_{tooth_label}_circle.json')
            )

        # if not all_teeth_saved:
        #     easy_mesh_with_gums = Easy_Mesh(filename=input_vtp_file)
        #     easy_mesh_with_gums.cell_attributes['Label'] = np.zeros([easy_mesh_with_gums.cell_ids.shape[0], 1])
        #
        #     # checking if the teeth are saved successfully or not
        #     for (i, tooth_label) in enumerate(teeth_labels):
        #         if saved_tooth_labels[tooth_label]:
        #             continue
        #
        #         # the tooth is not saved successfully
        #         logger.warning(f'Not saved {tooth_label=}')
        #         circle_id = circle_ids_from_left_to_right[i]
        #
        #         circle = tooth_circles[circle_id]
        #         circle_points = circle['points']
        #         circle_point_ids = circle['point_ids']
        #         circle_neighbour_point_ids = []
        #
        #         for point_id in circle_point_ids:
        #             # giving -1 label to the neighbour cells of the circle point ids
        #             for cell_id in mesh_with_gums_pkl.point_ids[point_id]['cell_ids']:
        #                 easy_mesh_with_gums.cell_attributes['Label'][cell_id] = -1
        #
        #             for neighbour_point_id in mesh_with_gums_pkl.point_ids[point_id]['neighbours']:
        #                 circle_neighbour_point_ids.append(neighbour_point_id)
        #
        #         n_circle_points = len(circle_points)
        #
        #         # finding the closest point of the circle center on the aligned mesh
        #         circle_center = np.mean(circle_points, axis=0)
        #         circle_center_on_mesh_with_gums_point_id = mesh_with_gums_pkl.find_closest_point_2d(
        #             point=circle_center,
        #             elevation_value=1,
        #             check_min_z_value=jaw_type == 'max'
        #         )
        #
        #         circle_center_on_mesh_with_gums_point = mesh_with_gums_pkl.get_point(
        #             point_id=circle_center_on_mesh_with_gums_point_id
        #         )
        #
        #         inner_points = {1: circle_center_on_mesh_with_gums_point}
        #         inner_points_count = 1
        #         inner_point_ids = [circle_center_on_mesh_with_gums_point_id]
        #
        #         n_extra_points_required = 10
        #         quarter_ratio = n_circle_points // n_extra_points_required
        #
        #         sink_point_ids_index = [i * quarter_ratio for i in range(n_extra_points_required)]
        #
        #         # finding inner points by taking the middle point of astar
        #         # paths from center to the multiple boundary points
        #         for index in sink_point_ids_index:
        #             path = mesh_with_gums_pkl.find_astar_path_2d(
        #                 source_point_id=circle_center_on_mesh_with_gums_point_id,
        #                 sink_point_id=circle_point_ids[index],
        #             )
        #
        #             point_id = path[len(path) // 2]
        #             tooth_point = mesh_with_gums_pkl.get_point(point_id)
        #
        #             inner_point_ids.append(point_id)
        #             inner_points[inner_points_count] = tooth_point
        #             inner_points_count += 1
        #
        #         circle_points_path = os.path.join(temp, f'Tooth_{tooth_label}_circles.json')
        #         inner_points_path = os.path.join(temp, f'Tooth_{tooth_label}_marked_points.json')
        #
        #         save_as_json(circle_points, circle_points_path)
        #         save_as_json(inner_points, inner_points_path)
        #
        #         grab_cut(
        #             mesh_with_gums_stl_path=input_aligned_mesh_stl_file,
        #             mesh_with_gums_pkl_path=input_pkl_file,
        #             mesh_with_gums_vtp_path=input_vtp_file,
        #             circle_points_json_path=circle_points_path,
        #             marked_points_json_path=inner_points_path,
        #             output_folder_path=out,
        #             temp_folder_path=temp,
        #             called_as_api=False
        #         )
        #
        #         # # now, giving labels to each cell on the path from the inner
        #         # # point id to the circle neighbours
        #         # tooth_points = copy.deepcopy(inner_points)
        #         # tooth_point_ids = copy.deepcopy(inner_point_ids)
        #         # for inner_point_id in inner_point_ids:
        #         #     for circle_point in circle_points:
        #         #         circle_point_id = mesh_with_gums_pkl.find_closest_point(circle_point)
        #         #         path = mesh_with_gums_pkl.find_astar_path_2d_with_boundary_check(
        #         #             source_point_id=inner_point_id,
        #         #             sink_point_id=circle_point_id,
        #         #             boundary_point_ids=unique_circle_neighbour_point_ids
        #         #         )
        #         #
        #         #         for point_id in path:
        #         #             tooth_points.append(mesh_with_gums_pkl.get_point(point_id))
        #         #             tooth_point_ids.append(point_id)
        #         #
        #         #             for cell_id in mesh_with_gums_pkl.point_ids[point_id]['cell_ids']:
        #         #                 easy_mesh_with_gums.cell_attributes['Label'][cell_id] = tooth_label
        #         #
        #         # tooth_stl_path = save_stl_from_easy_mesh(
        #         #     easy_mesh=easy_mesh_with_gums,
        #         #     temp=temp,
        #         #     out=out,
        #         #     cell_label=tooth_label,
        #         #     filename_without_extension=f'Tooth_{tooth_label}_before_filled_2'
        #         # )
        #         #
        #         # # fill the small holes using vedo function used below
        #         # clean_stl_path = os.path.join(temp, f'Tooth_{tooth_label}_clean_1.stl')
        #         # save_clean_mesh(tooth_stl_path, clean_stl_path)
        #         # filled_stl_path = os.path.join(out, f'Tooth_{tooth_label}.stl')
        #         # tooth_fill_hole(clean_stl_path, filled_stl_path)
        #         # continue
        #         #
        #         # # vedo_mesh = vedo.load(tooth_stl_path)
        #         # # filled_vedo_mesh = vedo_mesh.fillHoles(size=2)
        #         # # filled_vedo_mesh.write(filled_stl_path)
        #         #
        #         # teeth_easy_mesh = Easy_Mesh(filename=filled_stl_path)
        #         # o3d_tooth_mesh = o3d.io.read_triangle_mesh(filled_stl_path)
        #         # o3d_tooth_mesh.compute_vertex_normals()
        #         #
        #         # tooth_mesh_hole_points = teeth_easy_mesh.get_ordered_boundary_points()
        #         # stitch_together_open_boundary_points(tooth_mesh_hole_points)
        #         #
        #         # tooth_crown_boundary_id = -1
        #         # for (key, tooth_boundary_points) in tooth_mesh_hole_points.items():
        #         #     new_tooth_boundary_points = []
        #         #     for point in tooth_boundary_points:
        #         #         min_dist = np.min(np.linalg.norm(circle_points - point, axis=1))
        #         #         if min_dist < 0.5:
        #         #             continue
        #         #
        #         #         new_tooth_boundary_points.append(point)
        #         #
        #         #     tooth_mesh_hole_points[key] = np.asarray(new_tooth_boundary_points)
        #         #
        #         # logger.info(f'{tooth_mesh_hole_points.keys()=}')
        #         #
        #         # for (key, tooth_hole_boundary_points) in tooth_mesh_hole_points.items():
        #         #     n_hole_points = len(tooth_hole_boundary_points)
        #         #
        #         #     pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(tooth_hole_boundary_points))
        #         #     pcd.paint_uniform_color([1, 0, 0])
        #         #     o3d.visualization.draw_geometries([o3d_tooth_mesh, pcd])
        #         #
        #         #     if key == tooth_crown_boundary_id:
        #         #         continue
        #         #
        #         #     n_middle = n_hole_points // 2
        #         #
        #         #     tooth_boundary_point_ids = \
        #         #         [mesh_with_gums_pkl.find_closest_point(point) for point in tooth_hole_boundary_points]
        #         #
        #         #     for (j, point_id1) in enumerate(tooth_boundary_point_ids[:n_middle]):
        #         #         for (k, point_id2) in enumerate(tooth_boundary_point_ids[n_middle:]):
        #         #             path = mesh_with_gums_pkl.find_astar_path_2d(
        #         #                 source_point_id=point_id1,
        #         #                 sink_point_id=point_id2,
        #         #                 restrict_point_ids=tooth_point_ids
        #         #             )
        #         #
        #         #             for point_id in path:
        #         #                 tooth_points.append(mesh_with_gums_pkl.get_point(point_id))
        #         #
        #         #                 for cell_id in mesh_with_gums_pkl.point_ids[point_id]['cell_ids']:
        #         #                     easy_mesh_with_gums.cell_attributes['Label'][cell_id] = tooth_label
        #         #
        #         # tooth_stl_path = save_stl_from_easy_mesh(
        #         #     easy_mesh=easy_mesh_with_gums,
        #         #     temp=temp,
        #         #     out=out,
        #         #     cell_label=tooth_label,
        #         #     filename_without_extension=f'Tooth_{tooth_label}_vtp_after_filled'
        #         # )
        #         #
        #         # # fill the small holes using vedo function used below
        #         # clean_stl_path = os.path.join(temp, f'Tooth_{tooth_label}_clean_2.stl')
        #         # save_clean_mesh(tooth_stl_path, clean_stl_path)
        #         # filled_stl_path = os.path.join(out, f'Tooth_{tooth_label}.stl')
        #         # tooth_fill_hole(clean_stl_path, filled_stl_path)
        #         #
        #         # # vedo_mesh = vedo.load(tooth_stl_path)
        #         # # filled_vedo_mesh = vedo_mesh.fillHoles(size=2)
        #         # # filled_vedo_mesh.write(filled_stl_path)

        logger.info(f'Step 10: Output teeth STLs saved in time {time.time() - st}')
        end_time = time.time()
        logger.info(f'END: Total time taken: {end_time - start_time}')

        # if below check is False, then the next tool for saving the incisors is required
        if len(teeth_not_saved) != 0:
            logger.warning(f'Warning: Tooth labels {teeth_not_saved} not saved because of error in the scans. '
                           f'Their circles saved in this folder: {not_save_teeth_folder_path}')

            return 'DEFECTIVE_SCANS'

        return 'Success'

    except Exception as e:
        logging.error(e)
        tb = traceback.format_exc()
        logging.error(tb)
        return 'Error'


POLYGON_POINT_DISTANCE_THRESHOLD = 1
