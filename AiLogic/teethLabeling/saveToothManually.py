import copy
import errno
import logging
import os
import pickle
import time

import networkx as nx
import numpy as np
import open3d as o3d
import vedo

from AiLogic.teethLabeling.alignedMeshClass import AlignedTeethMesh
from AiLogic.teethLabeling.easyMeshClass import Easy_Mesh
from AiLogic.teethLabeling.utils import INCISORS_PREMOLARS_RANGE, get_line_object
from AiLogic.teethLabeling.utils import get_json_object, get_slope, get_plane_equation_from_points, \
    get_cos_of_angle_between_planes, save_stl_from_easy_mesh

logger = logging.getLogger(name='saveToothManually')

ITERATIONS_COUNT = 18


def save_tooth_manually(
        mesh_with_gums_stl_path: str,
        mesh_with_gums_vtp_path: str,
        mesh_with_gums_pkl_path: str,
        marked_points_json_path: str,
        circle_points_json_path: str,
        output_folder_path: str,
        temp_folder_path: str,
        show_visualisations: bool = False,
):
    """
    Apply astar for the teeth from circle points to the inner points marked on the UI

    @param mesh_with_gums_stl_path: Path of STL file of the aligned mesh
    @param mesh_with_gums_vtp_path: Path of VTP file of the aligned mesh
    @param mesh_with_gums_pkl_path: Path of PKL file of the aligned mesh
                                    (contains the info about the mesh points)
    @param marked_points_json_path: Path of JSON file of points marked by
                                    the user on the aligned mesh
    @param circle_points_json_path: Path of JSON file of circle points
    @param output_folder_path: Output folder path
    @param temp_folder_path: Temp folder path
    @param show_visualisations: This parameter should only be used while running
                                the script locally for visualizing the output

    @return: Saves the STL of the Tooth_{tooth_label}.stl
    """
    
    if not os.path.isfile(mesh_with_gums_vtp_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), mesh_with_gums_vtp_path)

    if not os.path.isfile(mesh_with_gums_pkl_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), mesh_with_gums_pkl_path)

    if not os.path.isfile(marked_points_json_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), marked_points_json_path)

    if not os.path.isfile(circle_points_json_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), circle_points_json_path)

    # create temp and out folders
    os.mkdir(output_folder_path)
    os.mkdir(temp_folder_path)

    start_time = time.time()

    tooth_label = int(circle_points_json_path.split('_')[-2])
    logger.info(f'{tooth_label=}')

    # starting the timer to calculate the time taken for teeth segmentation per case
    f = open(mesh_with_gums_pkl_path, 'rb')
    point_ids = pickle.load(f)
    mesh_with_gums_pkl = AlignedTeethMesh(point_ids)
    easy_mesh_with_gums = Easy_Mesh(filename=mesh_with_gums_vtp_path)
    easy_mesh_with_gums.cell_attributes['Label'] = np.zeros([easy_mesh_with_gums.cell_ids.shape[0], 1])

    logger.info(f'Read the input pkl file {mesh_with_gums_pkl_path}')

    # loading circle points json file
    circle_points = get_json_object(circle_points_json_path)

    o3d_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(mesh_with_gums_stl_path)
    o3d_mesh.compute_vertex_normals()

    pcd_circle_points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(circle_points))
    pcd_circle_points.paint_uniform_color([0, 0, 1])

    bounding_box_points = copy.deepcopy(circle_points)

    circle_point_ids = []
    circle_neighbour_point_ids = []

    for point in circle_points:
        point_id = mesh_with_gums_pkl.find_closest_point(point)
        circle_point_ids.append(point_id)

        for neighbour_point_id in mesh_with_gums_pkl.point_ids[point_id]['neighbours']:
            circle_neighbour_point_ids.append(neighbour_point_id)
            bounding_box_points.append(mesh_with_gums_pkl.get_point(neighbour_point_id))

    points = get_json_object(marked_points_json_path)
    marked_inner_points = []
    marked_inner_point_ids = []
    for (key, point) in points.items():
        inner_point = [point["_x"], point["_z"], point["_y"]]
        inner_point_id = mesh_with_gums_pkl.find_closest_point(inner_point)
        marked_inner_points.append(inner_point)
        marked_inner_point_ids.append(inner_point_id)

        for neighbour_point_id in mesh_with_gums_pkl.point_ids[inner_point_id]['neighbours']:
            bounding_box_points.append(mesh_with_gums_pkl.get_point(neighbour_point_id))

    if show_visualisations:
        pcd_mip = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(marked_inner_points))
        pcd_mip.paint_uniform_color([1, 0, 0])

        o3d.visualization.draw_geometries([o3d_mesh, pcd_circle_points, pcd_mip])

    astar_inner_point_ids = []
    for center_point_id in marked_inner_point_ids:
        for point_id in circle_point_ids:
            path = mesh_with_gums_pkl.find_straight_path(
                source_point_id=center_point_id,
                sink_point_id=point_id,
            )

            astar_inner_point_ids += path

    center_point_id = marked_inner_point_ids[0]
    for point_id in marked_inner_point_ids[1:]:
        path = mesh_with_gums_pkl.find_astar_path_2d(
            source_point_id=center_point_id,
            sink_point_id=point_id,
        )

        astar_inner_point_ids += path

    astar_inner_point_ids = list(np.unique(astar_inner_point_ids))
    astar_inner_points = []
    for pid in astar_inner_point_ids:
        astar_inner_points.append(mesh_with_gums_pkl.get_point(pid))

    if show_visualisations:
        pcd_astar_inner_points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(astar_inner_points))
        pcd_astar_inner_points.paint_uniform_color([1, 0, 0])

        o3d.visualization.draw_geometries([o3d_mesh, pcd_astar_inner_points, pcd_circle_points])

    point_ids = copy.deepcopy(circle_point_ids + marked_inner_point_ids)
    start = 0
    end = len(point_ids)
    visited_points = {}

    # level for loop
    for _ in range(3):
        for i in range(start, end):
            point_id = point_ids[i]
            visited_points[point_id] = 1

            for neighbour_point_id in mesh_with_gums_pkl.point_ids[point_id]['neighbours']:
                bounding_box_points.append(mesh_with_gums_pkl.get_point(neighbour_point_id))
                point_ids.append(neighbour_point_id)

        start = end
        end = len(point_ids)

    logger.info(f'Total no of marked inner points: {len(bounding_box_points)}')

    o3d_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(mesh_with_gums_stl_path)
    o3d_mesh.compute_vertex_normals()

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(bounding_box_points))
    pcd.paint_uniform_color([1, 0, 0])

    bounding_box: o3d.geometry.OrientedBoundingBox = o3d.geometry.OrientedBoundingBox.create_from_points(
        points=o3d.utility.Vector3dVector(bounding_box_points))
    bounding_box.color = (1, 0, 0)

    bounding_box = copy.deepcopy(bounding_box).scale(scale=1.15, center=bounding_box.get_center())
    bigger_bounding_box = copy.deepcopy(bounding_box).scale(scale=1.25, center=bounding_box.get_center())
    bigger_bounding_box.color = (0, 0, 1)

    logger.info(f'Get both the boundary boxes for cropping the aligned mesh')

    if show_visualisations:
        o3d.visualization.draw_geometries([o3d_mesh, bounding_box, bigger_bounding_box, pcd])

    bigger_mesh = o3d_mesh.crop(bigger_bounding_box)
    bigger_mesh.compute_vertex_normals()
    bigger_mesh_path = os.path.join(temp_folder_path, 'bigger_mesh.stl')

    if show_visualisations:
        o3d.visualization.draw_geometries([bigger_mesh, bounding_box, bigger_bounding_box])

    o3d.io.write_triangle_mesh(
        filename=bigger_mesh_path,
        mesh=bigger_mesh,
    )

    logger.info(f'Saving the bigger cropped mesh at path {bigger_mesh_path}')

    smaller_mesh = o3d_mesh.crop(bounding_box)
    smaller_mesh.compute_vertex_normals()
    smaller_mesh_path = os.path.join(temp_folder_path, 'smaller_mesh.stl')

    o3d.io.write_triangle_mesh(
        filename=smaller_mesh_path,
        mesh=smaller_mesh,
    )

    logger.info(f'Saving the smaller cropped mesh at path {smaller_mesh_path}')

    smaller_easy_mesh = Easy_Mesh(filename=smaller_mesh_path)
    smaller_easy_mesh_points = np.asarray(smaller_easy_mesh.points)
    inner_mesh_point_ids, inner_mesh_cell_ids = initialize_mesh(
        easy_mesh=smaller_easy_mesh,
        get_cell_neighbours=True
    )
    logger.info('Initialize smaller easy mesh')

    if show_visualisations:
        o3d_bigger_mesh = o3d.io.read_triangle_mesh(bigger_mesh_path)
        o3d_bigger_mesh.compute_vertex_normals()

    bigger_easy_mesh: Easy_Mesh = Easy_Mesh(filename=bigger_mesh_path)
    bigger_easy_mesh_points = np.asarray(bigger_easy_mesh.points)
    bigger_mesh_point_ids, bigger_mesh_cell_ids = initialize_mesh(
        easy_mesh=bigger_easy_mesh,
        get_cell_neighbours=True
    )
    logger.info('Initialize bigger easy mesh')

    circle_cell_ids_on_bigger_mesh = []
    for circle_point in circle_points:
        point_id = find_closest_point_float(
            point=circle_point,
            points_float=bigger_easy_mesh_points,
        )

        for cell_id in bigger_mesh_point_ids[point_id]['cell_ids']:
            bigger_mesh_cell_ids[cell_id]['is_tooth_boundary'] = True
            circle_cell_ids_on_bigger_mesh.append(cell_id)

    astar_cell_ids_on_bigger_mesh = []
    for astar_point in astar_inner_points:
        point_id = find_closest_point_float(
            point=astar_point,
            points_float=bigger_easy_mesh_points,
        )

        for cell_id in bigger_mesh_point_ids[point_id]['cell_ids']:
            bigger_mesh_cell_ids[cell_id]['is_tooth_boundary'] = True
            astar_cell_ids_on_bigger_mesh.append(cell_id)

    user_marked_peak_cell_ids = []
    user_marked_bridge_cell_ids = []
    for (i, inner_point) in enumerate(marked_inner_points):
        point_id = find_closest_point_float(
            point=inner_point,
            points_float=bigger_easy_mesh_points
        )

        if i < 3:
            for cell_id in bigger_mesh_point_ids[point_id]['cell_ids']:
                user_marked_peak_cell_ids.append(cell_id)

        else:
            for cell_id in bigger_mesh_point_ids[point_id]['cell_ids']:
                user_marked_bridge_cell_ids.append(cell_id)

    user_marked_source_cell_id = user_marked_peak_cell_ids[0]

    logger.info(f'Len of user marked cell ids {len(user_marked_peak_cell_ids) + len(user_marked_bridge_cell_ids)}')

    if tooth_label in INCISORS_PREMOLARS_RANGE:
        for cell_id in user_marked_bridge_cell_ids:
            bigger_mesh_cell_ids[cell_id]['is_tooth_boundary'] = True

        cell_ids = copy.deepcopy(user_marked_bridge_cell_ids)
        start = 0
        end = len(cell_ids)
        for level in range(3):
            for i in range(start, end):
                cell_id = cell_ids[i]
                for neighbour_cell_id in bigger_mesh_cell_ids[cell_id]['edge_neighbours']:
                    bigger_mesh_cell_ids[cell_id]['is_tooth_boundary'] = True
                    cell_ids.append(neighbour_cell_id)

            start = end
            end = len(cell_ids)

    barry_centers_cell_ids_mapping = []
    barry_centers = []

    ig_edges = []

    # initializing edges for networkx graph
    for (cell_id, mesh_cell) in bigger_mesh_cell_ids.items():
        mesh_cell['is_inside'] = False
        mesh_cell['distance_from_inner_boundary'] = -1
        cur_barry_center = mesh_cell['barry_center']
        barry_centers_cell_ids_mapping.append(cell_id)
        barry_centers.append(cur_barry_center)

        mesh_cell['edge_indices'] = {}

        for neighbour_cell_id in mesh_cell['edge_neighbours']:
            mesh_cell['center_ripple_values'][neighbour_cell_id] = 0

            # check if same neighbour cell has appeared twice or not
            if neighbour_cell_id in mesh_cell['edge_indices']:
                continue

            ig_edges.append((cell_id, neighbour_cell_id))
            mesh_cell['edge_indices'][neighbour_cell_id] = len(ig_edges) - 1

    logger.info('Initialized networkx graph edges')

    inner_mesh_bigger_mesh_cell_mapping = {}
    for (cell_id, mesh_cell) in inner_mesh_cell_ids.items():
        cur_barry_center = mesh_cell['barry_center']
        index = find_closest_point_float(point=cur_barry_center, points_float=np.asarray(barry_centers).astype(float))

        bigger_mesh_cell_id = barry_centers_cell_ids_mapping[index]
        bigger_mesh_cell_ids[bigger_mesh_cell_id]['is_inside'] = True
        bigger_mesh_cell_ids[bigger_mesh_cell_id]['inner_cell_id'] = cell_id
        inner_mesh_bigger_mesh_cell_mapping[cell_id] = bigger_mesh_cell_id

    logger.info(f'Mapping of bigger mesh and inner mesh cells is done')

    # lf
    cell_labels = {cell_id: 1 if mesh_cell['is_inside'] else 0 for (cell_id, mesh_cell) in bigger_mesh_cell_ids.items()}
    logger.info('Initialize cell labels')

    if show_visualisations:
        inside_cells = []
        outside_cells = []
        for (cell_id, mesh_cell) in bigger_mesh_cell_ids.items():
            if mesh_cell['is_inside']:
                inside_cells.append(mesh_cell['barry_center'])
            else:
                outside_cells.append(mesh_cell['barry_center'])

        pcd_inside = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(inside_cells))
        pcd_inside.paint_uniform_color([1, 1, 0])

        pcd_outside = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(outside_cells))
        pcd_outside.paint_uniform_color([0, 1, 0])

        o3d.visualization.draw_geometries([o3d_bigger_mesh, pcd_outside, pcd_inside, bounding_box, bigger_bounding_box])

    inner_boundary_points_dict = smaller_easy_mesh.get_ordered_boundary_points()
    inner_boundary_points = []
    for (key, boundary_points) in inner_boundary_points_dict.items():
        inner_boundary_points = inner_boundary_points + list(boundary_points)

    logger.info('Get the boundary points of the inner cropped mesh')

    inner_boundary_point_ids = []
    inner_boundary_cell_ids = []
    for point in inner_boundary_points:
        point_id = find_closest_point_float(
            point=point,
            points_float=smaller_easy_mesh_points
        )

        inner_boundary_point_ids.append(point_id)
        for cell_id in inner_mesh_point_ids[point_id]['cell_ids']:
            inner_boundary_cell_ids.append(cell_id)
            break

    logger.info(f'Found: {len(inner_boundary_cell_ids)=}')

    distance_from_boundary_box = {}
    for edge in ig_edges:
        if edge[0] in distance_from_boundary_box:
            distance_from_boundary_box[edge[0]][edge[1]] = 10000
        else:
            distance_from_boundary_box[edge[0]] = {
                edge[1]: 10000
            }

    cell_ids = copy.deepcopy(inner_boundary_cell_ids)
    start = 0
    end = len(cell_ids)

    max_path_distance = 0

    path_distances = []

    visited_cells = {}
    for cell_id in inner_boundary_cell_ids:
        visited_cells[cell_id] = 1

    while end > start:
        for i in range(start, end):
            parent_cell_id = cell_ids[i]

            parent_cell_id_in_bigger = inner_mesh_bigger_mesh_cell_mapping[parent_cell_id]
            parent_mesh_cell_in_bigger = bigger_mesh_cell_ids[parent_cell_id_in_bigger]

            for neighbour_cell_id in inner_mesh_cell_ids[parent_cell_id]['edge_neighbours']:
                if neighbour_cell_id in cell_ids:
                    continue

                neighbour_cell_id_in_bigger = inner_mesh_bigger_mesh_cell_mapping[neighbour_cell_id]
                neighbour_mesh_cell_in_bigger = bigger_mesh_cell_ids[neighbour_cell_id_in_bigger]

                cur_distance = 1 + parent_mesh_cell_in_bigger['distance_from_inner_boundary']

                if cur_distance > max_path_distance:
                    max_path_distance = cur_distance

                if -1 < neighbour_mesh_cell_in_bigger['distance_from_inner_boundary'] < cur_distance:
                    continue

                path_distances.append(cur_distance)

                neighbour_mesh_cell_in_bigger['distance_from_inner_boundary'] = cur_distance
                distance_from_boundary_box[parent_cell_id_in_bigger][neighbour_cell_id_in_bigger] = cur_distance

                if neighbour_cell_id in visited_cells:
                    continue

                visited_cells[neighbour_cell_id] = 1
                cell_ids.append(neighbour_cell_id)

        start = end
        end = len(cell_ids)

    logger.info(f'Calculated distance values for all inside cells from the boundary box')

    non_tooth_cell_id = 0
    for (cell_id, cell_label) in cell_labels.items():
        if cell_label == 0:
            non_tooth_cell_id = cell_id
            break

    if show_visualisations:
        source_point = bigger_mesh_cell_ids[non_tooth_cell_id]['barry_center']
        source_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([source_point]))
        source_pcd.paint_uniform_color([1, 0, 0])

        target_point = bigger_mesh_cell_ids[user_marked_source_cell_id]['barry_center']
        target_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([target_point]))
        target_pcd.paint_uniform_color([0, 0, 1])

        o3d.visualization.draw_geometries([o3d_bigger_mesh, source_pcd, target_pcd])

    logger.info(
        f'Applying graph min cut algo: Source cell id = {user_marked_source_cell_id}, Target cell id = {non_tooth_cell_id}')

    MINIMUM_EDGE_VALUE = 0
    MAXIMUM_EDGE_VALUE = 10000

    G = nx.Graph()
    # edge_lines = []

    cell_ids = copy.deepcopy([user_marked_source_cell_id])
    start = 0
    end = len(cell_ids)
    for level in range(2):
        for i in range(start, end):
            cell_id = cell_ids[i]
            for neighbour_cell_id in bigger_mesh_cell_ids[cell_id]['edge_neighbours']:
                bigger_mesh_cell_ids[cell_id]['center_ripple_values'][neighbour_cell_id] = MAXIMUM_EDGE_VALUE
                cell_ids.append(neighbour_cell_id)

        start = end
        end = len(cell_ids)

    for (i, edge) in enumerate(ig_edges):
        cell_id = edge[0]
        mesh_cell = bigger_mesh_cell_ids[cell_id]

        neighbour_cell_id = edge[1]
        neighbour_mesh_cell = bigger_mesh_cell_ids[neighbour_cell_id]

        if cell_labels[neighbour_cell_id] == 1:
            data_term = 1 + distance_from_boundary_box[cell_id][neighbour_cell_id] + mesh_cell['center_ripple_values'][
                neighbour_cell_id]
        else:
            data_term = MAXIMUM_EDGE_VALUE

        if mesh_cell['is_tooth_boundary'] and neighbour_mesh_cell['is_tooth_boundary']:
            G.add_edge(cell_id, neighbour_cell_id, capacity=MINIMUM_EDGE_VALUE)
            # edge_lines.append(
            #     get_line_object(
            #         point1=bigger_mesh_cell_ids[cell_id]['barry_center'],
            #         point2=bigger_mesh_cell_ids[neighbour_cell_id]['barry_center'],
            #         color=[1, 0, 0]
            #     )
            # )
        elif cell_id in astar_cell_ids_on_bigger_mesh and neighbour_cell_id in astar_cell_ids_on_bigger_mesh:
            G.add_edge(cell_id, neighbour_cell_id, capacity=MAXIMUM_EDGE_VALUE)

        else:
            G.add_edge(cell_id, neighbour_cell_id, capacity=data_term)
            # edge_lines.append(
            #     get_line_object(
            #         point1=bigger_mesh_cell_ids[cell_id]['barry_center'],
            #         point2=bigger_mesh_cell_ids[neighbour_cell_id]['barry_center'],
            #         color=[0, 0, data_term / max_path_distance]
            #     )
            # )

    # o3d.visualization.draw_geometries(edge_lines)

    # source cell id should be one of the cells of the foreground or tooth region
    # target cell id should be one of the cells of the background region
    cut_value, partition = nx.minimum_cut(G, _s=user_marked_source_cell_id, _t=non_tooth_cell_id)
    tooth_cells, non_tooth_cells = partition

    logger.info(f'{cut_value=}, Tooth cells: {len(tooth_cells)}, Other cells: {len(non_tooth_cells)}')

    circle_cells_reached_count = 0
    for cell_id in non_tooth_cells:
        cell_labels[cell_id] = 0
        if bigger_mesh_cell_ids[cell_id]['is_tooth_boundary']:
            circle_cells_reached_count += 1

    for cell_id in tooth_cells:
        cell_labels[cell_id] = 1

    final_tooth_cells = tooth_cells
    final_non_tooth_cells = non_tooth_cells

    if show_visualisations:
        bg_points = []
        for cell_id in final_non_tooth_cells:
            bg_points.append(bigger_mesh_cell_ids[cell_id]['barry_center'])

        fg_points = []
        for cell_id in final_tooth_cells:
            fg_points.append(bigger_mesh_cell_ids[cell_id]['barry_center'])

        bg_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(bg_points))
        bg_pcd.paint_uniform_color([1, 0, 0])
        fg_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(fg_points))
        fg_pcd.paint_uniform_color([0, 1, 0])

        o3d.visualization.draw_geometries([o3d_bigger_mesh, bg_pcd, fg_pcd, pcd_circle_points])

    user_marked_cell_ids = user_marked_peak_cell_ids + user_marked_bridge_cell_ids

    bigger_easy_mesh.cell_attributes['Label'] = np.zeros([bigger_easy_mesh.cell_ids.shape[0], 1])
    for cell_id in final_tooth_cells.union(set(user_marked_cell_ids)):
        bigger_easy_mesh.cell_attributes['Label'][cell_id] = 1

    for cell_id in circle_cell_ids_on_bigger_mesh:
        bigger_easy_mesh.cell_attributes['Label'][cell_id] = 0

    saved_stl_path = save_stl_from_easy_mesh(
        easy_mesh=bigger_easy_mesh,
        temp=temp_folder_path,
        out=output_folder_path,
        cell_label=1,
        filename_without_extension=f'Tooth_{tooth_label}_before_extraction',
        to_log=True,
    )

    to_extract_mesh: vedo.Mesh = vedo.load(saved_stl_path)
    tooth_mesh: vedo.Mesh = to_extract_mesh.extractLargestRegion()
    vedo.write(tooth_mesh, os.path.join(output_folder_path, f'Tooth_{tooth_label}.stl'))

    logger.info(f'Saved STL file for the tooth {tooth_label} in time: {time.time() - start_time}s')


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
            'distance_from_inner_boundary': -1,
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

        plane = get_plane_equation_from_points(points=[
            point1, point2, point3
        ])

        vector1 = get_slope(points=[point1, point2])
        vector2 = get_slope(points=[point1, point3])

        cell_normal = np.cross(vector1, vector2)
        cell_normal_length = np.linalg.norm(cell_normal)
        cell_normal /= cell_normal_length

        mesh_cell_ids[cell_id] = {
            'neighbours': set(),
            'edge_neighbours': set(),
            'barry_center': barry_center[0] / 3,
            'normal_vector': cell_normal,
            'plane_eq': plane,
            'is_tooth_boundary': False,
            'is_circle': False,
            'center_ripple_values': {},
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

        for (cell_id, cell_point_ids) in enumerate(easy_mesh.cell_ids):
            for neighbour_cell_id in mesh_cell_ids[cell_id]['neighbours']:
                neighbour_cell_point_ids = easy_mesh.cell_ids[neighbour_cell_id]
                if cell_point_ids[0] in neighbour_cell_point_ids and cell_point_ids[1] in neighbour_cell_point_ids:
                    mesh_cell_ids[cell_id]['edge_neighbours'] = mesh_cell_ids[cell_id]['edge_neighbours'].union(
                        {neighbour_cell_id})

                elif cell_point_ids[1] in neighbour_cell_point_ids and cell_point_ids[2] in neighbour_cell_point_ids:
                    mesh_cell_ids[cell_id]['edge_neighbours'] = mesh_cell_ids[cell_id]['edge_neighbours'].union(
                        {neighbour_cell_id})

                elif cell_point_ids[2] in neighbour_cell_point_ids and cell_point_ids[0] in neighbour_cell_point_ids:
                    mesh_cell_ids[cell_id]['edge_neighbours'] = mesh_cell_ids[cell_id]['edge_neighbours'].union(
                        {neighbour_cell_id})

    return mesh_point_ids, mesh_cell_ids


def find_closest_point_float(point, points_float):
    points = copy.deepcopy(points_float)
    new_point_id = np.argmin(np.linalg.norm(points - point, axis=1))
    return new_point_id
