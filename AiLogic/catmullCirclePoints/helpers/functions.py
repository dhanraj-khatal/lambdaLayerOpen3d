import argparse
import copy
import heapq
import os
import json
import math
import shutil
import logging
from math import pi
from typing import Union

import numpy as np
import open3d as o3d

from open3d.cpu.pybind.geometry import LineSet

from .constants import *

logger = logging.getLogger()


def get_json_object(filepath: str):
    # Opening JSON file
    f = open(filepath)

    # returns JSON object as a dictionary
    json_obj = json.load(f)

    # Closing file
    f.close()

    return json_obj


class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """

    def default(self, obj):
        if isinstance(obj, (np.int_, np.intc, np.intp, np.int8,
                            np.int16, np.int32, np.int64, np.uint8,
                            np.uint16, np.uint32, np.uint64)):
            return int(obj)
        elif isinstance(obj, (np.float_, np.float16, np.float32,
                              np.float64)):
            return float(obj)
        elif isinstance(obj, (np.ndarray,)):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def save_as_json(data, file_name):
    out_file = open(file_name, "w")
    json.dump(data, out_file, indent=4, cls=NumpyEncoder)
    out_file.close()


def get_args_parser(default_args=None) -> argparse.ArgumentParser:
    if default_args is None:
        default_args = {
            "case_folder": "",
            "json_folder": "",
        }

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--case-folder",
        "-cf",
        help="path to a particular case folder with initial teeth stls",
        default=default_args["case_folder"],
    )

    parser.add_argument(
        "--json-folder",
        "-jf",
        help="path to a particular json folder with details about root face, front face and center points of teeth crown",
        default=default_args["json_folder"],
    )

    return parser


def renew_folder(folder_path):
    if os.path.isdir(folder_path):
        shutil.rmtree(folder_path)

    os.makedirs(folder_path)


def get_euclidean_distance(sphere_center, sphere_goal_center):
    return np.sqrt(np.sum(np.square(sphere_goal_center - sphere_center)))


# sum of distances between x, y and z coordinates
def get_manhattan_distance(sphere_center, sphere_goal_center):
    return np.sum(np.abs(sphere_center - sphere_goal_center))


# FUNCTIONS
def get_distance_between_points_along_direction(point1: np.array, point2: np.array, direction: np.array):
    direction_magnitude = np.linalg.norm(direction)
    unit_direction = direction / direction_magnitude
    return np.dot(
        point1 - point2,
        unit_direction
    )


def convert_point_to_3d_team_format(point):
    return {
        "_x": point[0],
        "_y": point[2],
        "_z": point[1],
    }


def convert_point_to_3d_team_format1(point):
    return [point[0], point[2], point[1]]


def swap_z_and_y_coordinates(point: dict):
    return np.asarray([point["_x"], point["_z"], point["_y"]])


def vector_angle_about_normal_axis(u, v, n):
    # ref: https://stackoverflow.com/a/16544330
    dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
    det = u[0] * v[1] * n[2] + v[0] * n[1] * u[2] + n[0] * u[1] * v[2] - u[2] * v[1] * n[0] - v[2] * n[1] * u[0] - n[
        2] * u[1] * v[0]
    angle = math.atan2(det, dot)
    return np.round(angle, Constants.DECIMAL_PRECISION)


def vector_angle(u, v):
    x = (np.linalg.norm(u) * np.linalg.norm(v))
    if x == 0:
        x += 0.0001

    return np.arccos(np.dot(u, v) / x)


def draw_slope_angles(slope, xy_line=None, xz_line=None, yz_line=None, type="i"):
    if xy_line is None:
        xy_line = copy.deepcopy(slope)
        xy_line[2] = 0.0

        xz_line = copy.deepcopy(slope)
        xz_line[1] = 0.0

        yz_line = copy.deepcopy(slope)
        yz_line[0] = 0.0

    temp_points = np.vstack((xy_line, xz_line, yz_line, [0, 0, 0], slope))

    temp_lines = [
        [2, 3],  # x line
        [1, 3],
        [0, 3],
        # [3, 4]
    ]

    if type == "i":
        colors = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            # [0, 1, 1]
        ]
    else:
        colors = [
            [0.5, 0, 0],
            [0, 0.5, 0],
            [0, 0, 0.5],
            # [0, 0.5, 0.5]
        ]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(temp_points)
    line_set.lines = o3d.utility.Vector2iVector(temp_lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # c_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # o3d.visualization.draw_geometries([line_set, draw_line_end_points(temp_points), c_mesh])
    return line_set


def get_slope(points, i: int = 0, j: int = 1, to_norm: bool = True):
    line = np.asarray((points[j] - points[i]))
    if to_norm:
        line = line / (np.sqrt(np.sum(line ** 2)) + Constants.EPSILON)
    return line


def get_slope_angle_with_xyz(line):
    # z angle
    xy_line = copy.deepcopy(line)
    xy_line[2] = 0
    xy_plane_angle = (vector_angle(xy_line, [1, 0, 0])) / pi * 180
    xy_plane_angle = min(xy_plane_angle, 360 - xy_plane_angle)
    if line[1] < 0:
        xy_plane_angle = -1 * xy_plane_angle

    # y angle
    xz_line = copy.deepcopy(line)
    xz_line[1] = 0
    xz_plane_angle = (vector_angle(xz_line, [1, 0, 0])) / pi * 180
    xz_plane_angle = min(xz_plane_angle, 360 - xz_plane_angle)
    if line[2] > 0:
        xz_plane_angle = -1 * xz_plane_angle

    # x angle
    yz_line = copy.deepcopy(line)
    yz_line[0] = 0
    yz_plane_angle = (vector_angle(yz_line, [0, 1, 0])) / pi * 180
    yz_plane_angle = min(yz_plane_angle, 360 - yz_plane_angle)
    if line[2] < 0:
        yz_plane_angle = -1 * yz_plane_angle

    return [yz_plane_angle, xz_plane_angle, xy_plane_angle]


def get_angle_with_xyz(points, type="i"):
    line = get_slope(points)

    # z angle
    xy_line = copy.deepcopy(line)
    xy_line[2] = 0
    xy_plane_angle = (vector_angle(xy_line, [1, 0, 0])) / pi * 180
    if line[1] < 0:
        xy_plane_angle *= -1

    # y angle
    xz_line = copy.deepcopy(line)
    xz_line[1] = 0
    xz_plane_angle = (vector_angle(xz_line, [1, 0, 0])) / pi * 180
    if line[2] > 0:
        xz_plane_angle *= -1

    # x angle
    yz_line = copy.deepcopy(line)
    yz_line[0] = 0
    yz_plane_angle = (vector_angle(yz_line, [0, 1, 0])) / pi * 180
    if line[2] < 0:
        yz_plane_angle *= -1

    line_set = draw_slope_angles(line, xy_line=xy_line, xz_line=xz_line, yz_line=yz_line, type=type)

    return [yz_plane_angle, xz_plane_angle, xy_plane_angle], line_set


def get_angle_bw_lines(line1, line2):
    # z angle
    xy_line1 = copy.deepcopy(line1)
    xy_line1[2] = 0
    xy_line2 = copy.deepcopy(line2)
    xy_line2[2] = 0
    xy_plane_angle = (vector_angle(xy_line1, xy_line2)) / pi * 180

    # y angle
    xz_line1 = copy.deepcopy(line1)
    xz_line1[1] = 0
    xz_line2 = copy.deepcopy(line2)
    xz_line2[1] = 0
    xz_plane_angle = (vector_angle(xz_line1, xz_line2)) / pi * 180

    # x angle
    yz_line1 = copy.deepcopy(line1)
    yz_line1[0] = 0
    yz_line2 = copy.deepcopy(line2)
    yz_line2[0] = 0
    yz_plane_angle = (vector_angle(yz_line1, yz_line2)) / pi * 180

    return (vector_angle(line1, line2)) / pi * 180, yz_plane_angle, xz_plane_angle, xy_plane_angle


def get_angle_bw_line_and_coaxes(points, j=1, i=0):
    slope = points[j] - points[i]
    # draw_slope_angles(slope)
    z_angle = (vector_angle(slope, [1, 0, 0])) / pi * 180
    y_angle = (vector_angle(slope, [0, 1, 0])) / pi * 180
    x_angle = (vector_angle(slope, [0, 0, 1])) / pi * 180

    return slope, x_angle, y_angle, z_angle


def get_bounding_box_mid_points(
        bounding_box_vertices
):
    # Arranging plane in parallel and calculating there midpoint
    v1 = np.asarray(
        [
            bounding_box_vertices[0],
            bounding_box_vertices[1],
            bounding_box_vertices[3],
            bounding_box_vertices[6],
        ]
    )
    cv1 = v1.mean(axis=0)

    v2 = np.asarray(
        [
            bounding_box_vertices[2],
            bounding_box_vertices[4],
            bounding_box_vertices[5],
            bounding_box_vertices[7],
        ]
    )
    cv2 = v2.mean(axis=0)

    v3 = np.asarray(
        [
            bounding_box_vertices[0],
            bounding_box_vertices[2],
            bounding_box_vertices[3],
            bounding_box_vertices[5],
        ]
    )
    cv3 = v3.mean(axis=0)

    v4 = np.asarray(
        [
            bounding_box_vertices[1],
            bounding_box_vertices[4],
            bounding_box_vertices[6],
            bounding_box_vertices[7],
        ]
    )
    cv4 = v4.mean(axis=0)

    v5 = np.asarray(
        [
            bounding_box_vertices[3],
            bounding_box_vertices[4],
            bounding_box_vertices[5],
            bounding_box_vertices[6],
        ]
    )
    cv5 = v5.mean(axis=0)

    v6 = np.asarray(
        [
            bounding_box_vertices[0],
            bounding_box_vertices[1],
            bounding_box_vertices[2],
            bounding_box_vertices[7],
        ]
    )
    cv6 = v6.mean(axis=0)

    p = np.asarray([cv1, cv2, cv3, cv4, cv5, cv6])
    return p


def get_tooth_axes(
        mesh: Union[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh],
        factor: int = 5,
        color: int = 1
):
    bounding_box = mesh.get_oriented_bounding_box()

    # calculate box vertices
    bounding_box_vertices = np.array(bounding_box.get_box_points())

    # get tooth bounding box center
    ct = bounding_box.get_center()

    # Along Z axis extension for oriented box center point
    ct_up = copy.deepcopy(ct)
    ct_dwn = copy.deepcopy(ct)
    ct_up[1] = ct_up[1] + factor
    ct_dwn[1] = ct_dwn[1] - factor

    p = get_bounding_box_mid_points(bounding_box_vertices)
    ext_cor = copy.deepcopy(p)  # assigning  midpoint

    for j in range(len(ext_cor)):
        # calculating distance
        d = np.sqrt(np.sum((ct - ext_cor[j]) ** 2))
        # calculating slope
        slope = (ct - ext_cor[j]) / d
        # new extended coordinate
        ext_cor[j] = ext_cor[j] - (slope * factor)

    # # distance from midpoint coordinates with ct_up
    # d1 = []
    # for j in range(len(p)):
    #     d = np.sqrt(np.sum((ct_up - p[j]) ** 2))
    #     d1.append(d)
    #
    # i1 = d1.index(sorted(d1)[5])
    #
    # # distance from midpoints coordinates ct_dwn
    # d2 = []
    # for j in range(len(p)):
    #     d = np.sqrt(np.sum((ct_dwn - p[j]) ** 2))
    #     d2.append(d)
    #
    # i2 = d2.index(sorted(d2)[5])
    #
    # # arranged closest extended line with the axis
    # points = np.vstack(
    #     (
    #         ext_cor[i1],
    #         ext_cor[i2],  # root axis
    #         ct_up,
    #         ct_dwn,
    #     )
    # )

    points = np.vstack(
        (
            ext_cor[0],
            ext_cor[1],  # line 1
            ext_cor[2],
            ext_cor[3],  # line 2
            ext_cor[4],
            ext_cor[5],  # line 3
        )
    )

    lines = [
        [0, 1],
        [2, 3],
        [4, 5]
    ]

    if color == 1:
        colors = [
            [1, 0, 0],  # red
            [0, 1, 0],  # green
            [0, 0, 1],  # blue
        ]
    else:
        colors = [
            [0.5, 0, 0],  # red
            [0, 0.5, 0],  # green
            [0, 0, 0.5],  # blue
        ]

    tooth_axes = o3d.geometry.LineSet()
    tooth_axes.points = o3d.utility.Vector3dVector(points)
    tooth_axes.lines = o3d.utility.Vector2iVector(lines)
    tooth_axes.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([mesh, bounding_box, tooth_axes, draw_line_end_points(ext_cor), c_mesh])

    return tooth_axes


def draw_line_end_points(points, color=None):
    if color is None:
        color = [0, 0, 1]

    mid_red = o3d.geometry.PointCloud()
    mid_red.points = o3d.utility.Vector3dVector(points)
    mid_red.paint_uniform_color(color)
    return mid_red


def get_angle_bw_root_and_co_axes(
        lineset: LineSet,
):
    # root axis points
    points = lineset.get_line_coordinate(0)
    points_diff = points[0] - points[1]

    xy_hypotenuse = ((points_diff[0] ** 2) + points_diff[1] ** 2) ** 0.5
    yz_hypotenuse = ((points_diff[1] ** 2) + points_diff[2] ** 2) ** 0.5

    x_angle = y_angle = z_angle = 0
    if xy_hypotenuse > 0:
        x_angle = np.cosh(points_diff[0] / xy_hypotenuse)
        y_angle = np.cosh(points_diff[1] / xy_hypotenuse)

    if yz_hypotenuse > 0:
        z_angle = np.cosh(points_diff[2] / yz_hypotenuse)

    return x_angle, y_angle, z_angle


def create_video_for_demo(objects, filename="NA", camera_position: str = "front", center=None):
    # Create a renderer with the desired image size
    if center is None:
        center = [0, 0, 0]

    img_width = 1600
    img_height = 900
    renderer = o3d.visualization.rendering.OffscreenRenderer(img_width, img_height)

    # logger.info('Renderer created')

    # X is red, Y is green and Z is blue.
    renderer.scene.show_axes(True)

    # from front view, axis going left : -Y, axis going up : +Z, axis coming out : -X

    if camera_position == "lower_top":
        # Look at the origin from the front (along the -Z direction, into the screen), with Y as Up.
        eye = [center[0] - 50, center[1], center[2] + 200]  # camera position
        center = center  # look_at target
        up = [1, 0, 0]  # camera orientation
        # up = [0, 1, 0]  # camera orientation

    # checked for current alignment with X axis coming outside
    elif camera_position == "front_view":
        eye = [center[0] - 150, center[1] + 0, center[2] + 40]  # camera position
        center = center  # look_at target
        up = [0, 0, 1]  # camera orientation

    # checked for current alignment with X axis coming outside
    elif camera_position == "left_side_view":
        eye = [center[0] + 0, center[1] - 200, center[2] + 0]  # camera position
        center = center  # look_at target
        up = [0, 0, 1]  # camera orientation

    # checked for current alignment with X axis coming outside
    elif camera_position == "right_side_view":
        eye = [center[0] + 0, center[1] + 200, center[2] + 0]  # camera position
        center = center  # look_at target
        up = [0, 0, 1]  # camera orientation

    # checked for current alignment with X axis coming outside
    elif camera_position == "mandibular_view":
        eye = [center[0] - 110, center[1] + 100, center[2] - 250]  # camera position
        center = center  # look_at target
        up = [0, -1, 0]  # camera orientation

    elif camera_position == "maxillary_view":
        eye = [center[0] - 110, center[1] + 100, center[2] + 250]  # camera position
        center = center  # look_at target
        up = [0, -1, 0]  # camera orientation

    else:  # upper_top
        # Look at the origin from the front (along the -Z direction, into the screen), with Y as Up.
        eye = [center[0] - 50, center[1], center[2] - 200]  # camera position
        center = center  # look_at target
        up = [-1, 0, 0]  # camera orientation

    renderer.scene.camera.look_at(center, eye, up)
    # (The base color does not replace the arrows' own colors.)

    mtl = o3d.cpu.pybind.visualization.rendering.MaterialRecord()
    mtl.base_color = [1, 1, 1, 1.0]  # RGBA
    mtl.shader = "defaultUnlit"
    # logger.info('Create material record')

    for i, obj in enumerate(objects):
        if obj.is_dummy:
            continue

        obj.tooth.compute_vertex_normals()

        tooth_pcd = o3d.geometry.PointCloud()  # create a empty point cloud object
        tooth_pcd.points = obj.tooth.vertices  # add the points

        # tooth_pcd.paint_uniform_color([random.uniform(0, 0.7), random.uniform(0, 0.7), random.uniform(0, 0.7)])
        tooth_pcd.paint_uniform_color([0.7, 0.7, 0.7])
        renderer.scene.add_geometry("obj" + str(i), obj.tooth, mtl)
        renderer.scene.add_geometry("objpcd" + str(i), tooth_pcd, mtl)
        # renderer.scene.add_geometry("objgoal" + str(i), obj.goal_tooth, mtl)

    # logger.info('Add geometries')

    # Optionally set the camera field of view (to zoom in a bit)
    vertical_field_of_view = 15.0  # between 5 and 90 degrees
    aspect_ratio = img_width / img_height  # azimuth over elevation
    near_plane = 10
    far_plane = 1000.0
    fov_type = o3d.visualization.rendering.Camera.FovType.Vertical

    renderer.scene.camera.set_projection(vertical_field_of_view, aspect_ratio, near_plane, far_plane, fov_type)

    # logger.info('Set projection')

    # Read the image into a variable
    img_o3d = renderer.render_to_image()

    # logger.info('Render to image')

    o3d.io.write_image(filename, img_o3d, 9)

    logger.info(f'Write image {camera_position}')


def draw_objects_for_demo(objects, points=None, lines=None, highlight_tooth=-1, show_details=0,
                          camera_position: str = "front", to_show=None):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    if to_show is None:
        to_show = [0, 32]

    if lines is None:
        lines = []

    if points is not None:
        vis.add_geometry(draw_line_end_points(points))

    for line in lines:
        vis.add_geometry(line)

    for i in range(len(objects)):
        if objects[i].is_dummy:
            continue

        # for teeth beautification
        objects[i].tooth.compute_vertex_normals()
        objects[i].tooth.paint_uniform_color([0.7, 0.7, 0.7])

        if i == highlight_tooth:
            objects[i].tooth.paint_uniform_color([1, 0, 0])

        vis.add_geometry(objects[i].tooth)

        if show_details:
            c_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=7.0, origin=objects[i].tooth.get_center())
            vis.add_geometry(c_mesh)

    control = vis.get_view_control()
    control.change_field_of_view(60.0)
    control.set_zoom(0.8)

    if camera_position == "lower_top":
        # lower
        control.set_front([0, 0, 15])
        control.set_lookat([0, 10, 0])
        control.set_up([0, 1, 0])

    elif camera_position == "front":
        # front
        # control.set_front([0, -30, 0])
        control.set_front([-30, 0, 10])
        control.set_lookat([0, 0, 0])
        control.set_up([0, 1, 0])

    elif camera_position == "left":
        # left-side
        control.set_front([10, 10, -70])
        control.set_lookat([0, 0, 0])
        control.set_up([0, 1, 0])

    elif camera_position == "right":
        # right-side
        control.set_front([50, 0, 0])
        control.set_lookat([0, 0, 0])
        control.set_up([0, 0, 1])
    else:
        # upper
        control.set_front([0, 0, -20])
        control.set_lookat([0, 0, -20])
        control.set_up([0, -1, 0])

    # control.set_zoom(0.5)

    vis.run()
    vis.destroy_window()


def h(angle):
    return (angle / 180) - 1


def get_actual_root_point_max_distance_nn(tooth, crown_point_threshold, nearest_point):
    tooth_center = tooth.get_center()

    # merges the points which have distance between them less than 0.5
    # alpha_mesh = tooth.merge_close_vertices(0.2)
    # points1 = np.asarray(tooth.vertices)

    pcd = tooth.sample_points_poisson_disk(int(200))
    points1 = np.asarray(pcd.points)

    max_distance_from_center = 0
    min_distance_from_center = 1000000
    min_point = 100000

    for j, point1 in enumerate(points1):
        dist = get_euclidean_distance(point1, tooth_center)
        max_distance_from_center = max(dist, max_distance_from_center)
        min_distance_from_center = min(dist, min_distance_from_center)

        dist_from_nearest_point = get_euclidean_distance(point1, nearest_point)
        min_point = min(min_point, dist_from_nearest_point)

    # if crown_point_threshold < 0:
    #     max_distance_threshold = (
    #         max_distance_from_center + crown_point_threshold, max_distance_from_center)
    # else:
    #     max_distance_threshold = (
    #         min_distance_from_center, min_distance_from_center + crown_point_threshold)

    max_distance_threshold = (
        max_distance_from_center + crown_point_threshold, max_distance_from_center)

    far_points = []
    for j, point1 in enumerate(points1):
        dist = get_euclidean_distance(point1, tooth_center)
        dist_from_nearest_point = get_euclidean_distance(point1, nearest_point)
        if dist_from_nearest_point > min_point + 3:
            continue

        if max_distance_threshold[0] <= dist <= max_distance_threshold[1]:  # for molar teeth
            far_points.append(point1)

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(far_points))
    o3d.visualization.draw_geometries([tooth, pcd,
                                       o3d.geometry.TriangleMesh.create_coordinate_frame(size=5.0,
                                                                                         origin=nearest_point),
                                       o3d.geometry.TriangleMesh.create_coordinate_frame(origin=pcd.get_center())])
    return pcd.get_center()


def get_tooth_points_max_distance_nn(tooth, crown_point_threshold, nearest_point,
                                     dist_threshold_from_nearest_point: int = 3):
    tooth_center = tooth.get_center()

    # merges the points which have distance between them less than 0.5
    # alpha_mesh = tooth.merge_close_vertices(0.2)
    # points1 = np.asarray(tooth.vertices)

    pcd = tooth.sample_points_poisson_disk(int(200))
    points1 = np.asarray(pcd.points)

    max_distance_from_center = 0
    min_distance_from_center = 1000000
    min_point = 100000

    for j, point1 in enumerate(points1):
        dist = get_euclidean_distance(point1, tooth_center)
        max_distance_from_center = max(dist, max_distance_from_center)
        min_distance_from_center = min(dist, min_distance_from_center)

        dist_from_nearest_point = get_euclidean_distance(point1, nearest_point)
        min_point = min(min_point, dist_from_nearest_point)

    if crown_point_threshold < 0:
        max_distance_threshold = (
            max_distance_from_center + crown_point_threshold, max_distance_from_center)
    else:
        max_distance_threshold = (
            min_distance_from_center, min_distance_from_center + crown_point_threshold)

    far_points = []
    for j, point1 in enumerate(points1):
        dist = get_euclidean_distance(point1, tooth_center)
        dist_from_nearest_point = get_euclidean_distance(point1, nearest_point)
        if dist_from_nearest_point > min_point + dist_threshold_from_nearest_point:
            continue

        if max_distance_threshold[0] <= dist <= max_distance_threshold[1]:  # for molar teeth
            far_points.append(point1)

    return far_points


def get_actual_tooth_points_max_distance(tooth, crown_point_threshold):
    tooth_center = tooth.get_center()

    # merges the points which have distance between them less than 0.5
    # alpha_mesh = tooth.merge_close_vertices(0.4)
    # alpha_mesh = tooth.merge_close_vertices(0.2)
    # points1 = np.asarray(tooth.vertices)
    pcd = tooth.sample_points_poisson_disk(int(200))
    points1 = np.asarray(pcd.points)

    max_distance_from_center = 0
    min_distance_from_center = 1000000

    for j, point1 in enumerate(points1):
        dist = get_euclidean_distance(point1, tooth_center)
        if dist > max_distance_from_center:
            max_distance_from_center = dist

        if dist < min_distance_from_center:
            min_distance_from_center = dist

    if crown_point_threshold < 0:
        max_distance_threshold = (
            max_distance_from_center + crown_point_threshold, max_distance_from_center)
    else:
        max_distance_threshold = (
            min_distance_from_center, min_distance_from_center + crown_point_threshold)

    far_points = []
    for j, point1 in enumerate(points1):
        dist = get_euclidean_distance(point1, tooth_center)

        if max_distance_threshold[0] <= dist <= max_distance_threshold[1]:  # for molar teeth
            far_points.append(point1)

    return far_points


def get_point_on_plane_nearest_to_another_point(plane, point):
    t = (plane[3] - (plane[0] * point[0]) - (plane[1] * point[1]) - (plane[2] * point[2])) / (np.sum(point ** 2))
    point_on_plane = [
        plane[0] * t + point[0],
        plane[1] * t + point[1],
        plane[2] * t + point[2],
    ]

    return point_on_plane


def get_root_axis_from_bbox(
        mesh: Union[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh],
        factor: int = 5,
        color: int = 1
):
    bounding_box = mesh.get_oriented_bounding_box()

    # calculate box vertices
    bounding_box_vertices = np.array(bounding_box.get_box_points())

    # get tooth bounding box center
    tooth_center = bounding_box.get_center()

    p = get_bounding_box_mid_points(bounding_box_vertices)
    ext_cor = copy.deepcopy(p)  # assigning  midpoint

    for j in range(len(ext_cor)):
        # calculating distance
        d = np.sqrt(np.sum((tooth_center - ext_cor[j]) ** 2))
        # calculating slope
        slope = (tooth_center - ext_cor[j]) / d
        # new extended coordinate
        ext_cor[j] = ext_cor[j] - (slope * factor)

    root_axis = get_slope(np.asarray((tooth_center, ext_cor[3])))
    plane_perpendicular_to_root_axis = get_plane_equation_from_normal(root_axis, tooth_center)

    point = copy.deepcopy(tooth_center)
    point[0] += 1
    point_on_plane = get_point_on_plane_nearest_to_another_point(
        plane_perpendicular_to_root_axis, point
    )

    # perpendicular_axis = np.cross(root_axis, tooth_center)
    # perpendicular_axis = perpendicular_axis / np.sqrt(np.sum(perpendicular_axis ** 2))
    perpendicular_axis = get_slope([point_on_plane, tooth_center])

    perpendicular_point_1 = tooth_center + (perpendicular_axis * 2 * factor)
    perpendicular_point_2 = tooth_center - (perpendicular_axis * 2 * factor)

    points = np.vstack(
        (
            ext_cor[2],
            ext_cor[3],
            perpendicular_point_1,
            perpendicular_point_2
        )
    )

    lines = [
        [0, 1],
        [2, 3]
    ]

    if color == 1:
        colors = [
            [0, 1, 0],  # green
            [1, 0, 0],  # green
        ]
    else:
        colors = [
            [0, 0.5, 0],  # green
            [0.5, 0, 0],  # green
        ]

    root_axis = o3d.geometry.LineSet()
    root_axis.points = o3d.utility.Vector3dVector(points)
    root_axis.lines = o3d.utility.Vector2iVector(lines)
    root_axis.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([mesh, bounding_box, root_axis, draw_line_end_points(ext_cor), c_mesh])

    return root_axis


def get_root_axis_from_root_point(
        tooth_center, root_point,
        factor: int = 5,
        color: int = 1
):
    root_axis = get_slope(np.asarray((tooth_center, root_point)))
    root_point = root_point + (root_axis * factor)

    plane_perpendicular_to_root_axis = get_plane_equation_from_normal(root_axis, tooth_center)

    point = copy.deepcopy(tooth_center)
    point[0] += 1
    point_on_plane = get_point_on_plane_nearest_to_another_point(
        plane_perpendicular_to_root_axis, point
    )

    # perpendicular_axis = np.cross(root_axis, tooth_center)
    # perpendicular_axis = perpendicular_axis / np.sqrt(np.sum(perpendicular_axis ** 2))
    perpendicular_axis = get_slope([point_on_plane, tooth_center])

    perpendicular_point_1 = tooth_center + (perpendicular_axis * 2 * factor)
    # perpendicular_point_2 = tooth_center - (perpendicular_axis * 2 * factor)

    points = np.vstack(
        (
            tooth_center,
            root_point,
            perpendicular_point_1,
        )
    )

    lines = [
        [0, 1],
        [0, 2]
    ]

    if color == 1:
        colors = [
            [0, 1, 0],  # green
            [1, 0, 0],  # green
        ]
    else:
        colors = [
            [0, 0.5, 0],  # green
            [0.5, 0, 0],  # green
        ]

    root_axis = o3d.geometry.LineSet()
    root_axis.points = o3d.utility.Vector3dVector(points)
    root_axis.lines = o3d.utility.Vector2iVector(lines)
    root_axis.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([mesh, bounding_box, root_axis, draw_line_end_points(ext_cor), c_mesh])

    return root_axis


def get_plane_equation_from_normal(normal, point):
    d = np.dot(normal, point)
    plane = [normal[0], normal[1], normal[2], d]
    return plane


def get_point_distance_from_plane(plane, point):
    return abs((plane[0] * point[0]) + (plane[1] * point[1]) + (plane[2] * point[2]) - plane[3]) / np.sqrt(
        np.sum(np.square(plane[:3])))


def get_random_point_on_plane(plane):
    x = y = 3
    z = (plane[3] - (plane[0] * x + plane[1] * y)) / plane[2]
    return [x, y, z]


def get_plane_equation_from_points(points):
    A = points[0]
    B = points[1]
    C = points[2]

    a = ((B[1] - A[1]) * (C[2] - A[2])) - ((C[1] - A[1]) * (B[2] - A[2]))
    b = ((B[2] - A[2]) * (C[0] - A[0])) - ((C[2] - A[2]) * (B[0] - A[0]))
    c = ((B[0] - A[0]) * (C[1] - A[1])) - ((C[0] - A[0]) * (B[1] - A[1]))

    slope = np.asarray([a, b, c])
    slope = slope / (np.sqrt(np.sum(np.square(slope))) + Constants.EPSILON)
    a = slope[0]
    b = slope[1]
    c = slope[2]

    d = (a * A[0] + b * A[1] + c * A[2])

    return [a, b, c, d]


def get_plane_normal(plane):
    line = np.asarray(plane[:3])
    line = line / (np.sqrt(np.sum(line ** 2)) + Constants.EPSILON)
    return line


def check_point_location_wrt_plane(plane, point):
    return (plane[0] * point[0]) + (plane[1] * point[1]) + (plane[2] * point[2]) - plane[3]


def get_other_jaw_tooth_neighbours_from_center(
        objects,  # teeth objects
        tooth_id,  # tooth id for finding neighbours
        teeth_range,  # range of teeth ids for finding neighbours in
        lower_idx: int = 1,
        upper_idx: int = 33,
):
    i = tooth_id - lower_idx
    if objects[i].tooth is None:
        return []

    md = [1000, 1000, 1000]
    p = [-1, -1, -1]
    for j in teeth_range:
        if objects[j].tooth is None:
            continue

        d = get_euclidean_distance(objects[i].tooth.get_center(), objects[j].tooth.get_center())
        if d < md[0]:
            md[2] = md[1]
            md[1] = md[0]
            md[0] = d

            p[2] = p[1]
            p[1] = p[0]
            p[0] = j + 1
        elif d < md[1]:
            md[2] = md[1]
            md[1] = d

            p[2] = p[1]
            p[1] = j + 1
        elif d < md[2]:
            md[2] = d

            p[2] = j + 1

    return p


def get_tooth_distance(tooth_1, tooth_2):
    points = 100
    tooth_1_point_cloud = tooth_1.sample_points_poisson_disk(int(points))
    tooth_2_point_cloud = tooth_2.sample_points_poisson_disk(int(points))

    min_dist = 10000
    tooth_1_points = np.asarray(tooth_1_point_cloud.points)
    tooth_2_points = np.asarray(tooth_2_point_cloud.points)
    for i, tooth_1_point in enumerate(tooth_1_points):
        for j, tooth_2_point in enumerate(tooth_2_points):
            dist = get_euclidean_distance(tooth_1_point, tooth_2_point)
            min_dist = min(dist, min_dist)

    return min_dist


def get_other_jaw_tooth_neighbours_from_boundary(
        objects,  # teeth objects
        tooth_id,  # tooth id for finding neighbours
        teeth_range,  # range of teeth ids for finding neighbours in
        lower_idx: int = 1,
        upper_idx: int = 33,
        n_neighbours: int = 3
):
    i = tooth_id - lower_idx
    if objects[i].tooth is None:
        return []

    md = [1000, 1000, 1000]
    p = [-1, -1, -1]
    for j in teeth_range:
        if objects[j].tooth is None:
            continue

        d = get_tooth_distance(objects[i].tooth, objects[j].tooth)
        if d < md[0]:
            md[2] = md[1]
            md[1] = md[0]
            md[0] = d

            p[2] = p[1]
            p[1] = p[0]
            p[0] = j + 1
        elif d < md[1]:
            md[2] = md[1]
            md[1] = d

            p[2] = p[1]
            p[1] = j + 1
        elif d < md[2]:
            md[2] = d

            p[2] = j + 1

    return p


def get_tooth_neighbours(
        teeth_objects,
        show_lower_idx: int = Constants.LOWER_IDX,
        show_upper_idx: int = Constants.UPPER_IDX
):
    for tooth_id in range(show_lower_idx, show_upper_idx):
        i = tooth_id - Constants.LOWER_IDX
        if teeth_objects[i].tooth is None:
            continue

        r = Constants.MANDIBULAR_RANGE
        if i > 15:
            r = Constants.MAXILLARY_RANGE

        # get the top 3 neighbours in the other jaw of current tooth
        teeth_objects[i].other_jaw_neighbours = get_other_jaw_tooth_neighbours_from_boundary_using_heap(
            objects=teeth_objects,
            tooth_id=tooth_id,
            teeth_range=r,
            n_neighbours=3,
        )


def get_other_jaw_tooth_neighbours_from_boundary_using_heap(
        objects,  # teeth objects
        tooth_id,  # tooth id for finding neighbours
        teeth_range,  # range of teeth ids for finding neighbours in
        lower_idx: int = 1,
        upper_idx: int = 33,
        n_neighbours: int = 3
):
    i = tooth_id - lower_idx
    if objects[i].tooth is None:
        return []

    md = []
    for j in teeth_range:
        if objects[j].tooth is None:
            continue

        d = get_tooth_distance(objects[i].tooth, objects[j].tooth)
        if d > 4:
            continue

        heapq.heappush(md, (d, j + 1))

    n_nearest = heapq.nsmallest(n_neighbours, md)

    return [element[1] for element in n_nearest]


def get_line_segment_between_points(
        point1,
        point2,
        color=None  # blue
):
    if color is None:
        color = [0, 0, 1]

    points = np.vstack(
        (
            point1,
            point2,
        )
    )

    lines = [
        [0, 1],
    ]

    colors = [
        color,  # blue
    ]

    line_segment = o3d.geometry.LineSet()
    line_segment.points = o3d.utility.Vector3dVector(points)
    line_segment.lines = o3d.utility.Vector2iVector(lines)
    line_segment.colors = o3d.utility.Vector3dVector(colors)

    return line_segment


# helper function
def get_12_equidistant_circle_points_min_threshold(
        points: np.array,
        no_of_equidistant_points_required: int
) -> np.array:
    """
    Takes in a list of points to return a set of equidistant points of given length

    @param points: List of points
    @param no_of_equidistant_points_required: no of equidistant points required
    @return:
    """
    points = np.append(points, [points[0]], axis=0)
    n_points = len(points)
    pair_distances = []
    first_point = np.asarray(points[0])
    new_points = [first_point]

    for i in range(1, n_points - 1):
        second_point = np.asarray(points[i])
        dist = np.linalg.norm(second_point - first_point)
        print('dist', dist)

        if dist < 0.25:
            continue

        pair_distances.append(dist)
        new_points.append(second_point)
        first_point = second_point

    # total_circle_distance_length = np.sum(np.asarray(pair_distances))
    n_points = len(new_points)
    points = np.asarray(new_points)

    gap = n_points // no_of_equidistant_points_required
    equidistant_points = []

    for i in range(no_of_equidistant_points_required):
        equidistant_points.append(points[i * gap])

    return equidistant_points

    # current_segment_length = 0
    # for i in range(n_points - 1):
    #     new_segment_length = current_segment_length + pair_distances[i]
    #     if new_segment_length < required_segment_length:
    #         current_segment_length = new_segment_length
    #         continue
    #     else:
    #         if abs(current_segment_length - required_segment_length) < abs(
    #                 new_segment_length - required_segment_length):
    #             equidistant_points.append(list(points[i]))
    #             current_segment_length = pair_distances[i]
    #         else:
    #             equidistant_points.append(list(points[i + 1]))
    #             current_segment_length = 0
    #
    # return equidistant_points[no_of_equidistant_points_required-1::-1]


def get_equidistant_circle_points(
        points: np.array,
        no_of_equidistant_points_required: int
) -> np.array:
    """
    Takes in a list of points to return a set of equidistant points of given length

    @param points: List of points
    @param no_of_equidistant_points_required: no of equidistant points required
    @return:
    """
    points = np.append(points, [points[0]], axis=0)
    n_points = len(points)
    pair_distances = []

    for i in range(0, n_points):
        first_point = np.asarray(points[i])
        second_point = np.asarray(points[(i + 1) % n_points])

        dist = np.linalg.norm(second_point - first_point)
        pair_distances.append(dist)

    total_circle_distance_length = np.sum(np.asarray(pair_distances))
    equidistant_points = []
    for j in range(5):
        required_segment_length = total_circle_distance_length / (no_of_equidistant_points_required + j)
        equidistant_points = [points[0]]

        current_segment_length = 0
        for i in range(n_points - 1):
            new_segment_length = current_segment_length + pair_distances[i]
            if new_segment_length < required_segment_length:
                current_segment_length = new_segment_length
                continue

            if abs(current_segment_length - required_segment_length) < abs(
                    new_segment_length - required_segment_length):
                equidistant_points.append(list(points[i]))
                current_segment_length = pair_distances[i]
            else:
                equidistant_points.append(list(points[i + 1]))
                current_segment_length = 0

        print(f'j: {j}, len: {len(equidistant_points)}')
        if len(equidistant_points) >= no_of_equidistant_points_required:
            break

    return equidistant_points[no_of_equidistant_points_required-1::-1]


def smoothen_circle_points(
        points: np.array,
        threshold: float = 0.25
) -> np.array:
    """
    Takes in a list of points to return a set of equidistant points of given length

    @param points: List of points
    @param threshold:
    @return:
    """
    n_points = len(points)
    new_points = [points[0]]
    prev_point = np.asarray(points[0])

    for i in range(1, n_points - 1):
        cur_point = np.asarray(points[i])

        dist = np.linalg.norm(cur_point - prev_point)
        if dist < threshold:
            continue

        new_points.append(cur_point)
        prev_point = cur_point

    return np.asarray(new_points)


def project_points_on_to_xz_plane(
        points: np.array
) -> np.array:
    """
    Takes in a list of points to return a set of equidistant points of given length

    @param points: List of points
    @return:
    """
    n_points = len(points)
    new_points = []

    for i in range(0, n_points):
        first_point = np.asarray(points[i])
        new_points.append([first_point[0], first_point[1], 0])

    return np.asarray(new_points)


def get_avg_point(points: np.array) -> np.array:
    """
    Takes in a list of points to return a set of equidistant points of given length

    @param points: List of points
    @return:
    """
    return np.mean(points, axis=0)
    # n_points = len(points)
    # new_points = np.asarray(points[0])
    #
    # for i in range(1, n_points):
    #     first_point = np.asarray(points[i])
    #     new_points += first_point
    #
    # return new_points/n_points


def move_points(
        points: np.array,
        x: float,
        y: float,
        z: float
) -> np.array:
    """
    Takes in a list of points to return a set of equidistant points of given length

    @param points: List of points
    @param x: x
    @param y: y
    @param z: z
    @return:
    """
    n_points = len(points)
    new_points = []

    for i in range(0, n_points):
        first_point = np.asarray(points[i])
        new_points.append([first_point[0] + x, first_point[1] + y, first_point[2] + z])

    return np.asarray(new_points)


def get_intersection_of_line_with_circle(
        circle_center: np.array,
        rigged_gum_point: np.array,
        circle_points: np.ndarray
):
    line_slope = get_slope(points=[circle_center, rigged_gum_point])
    min_dist = 1000
    min_dist_index = -1

    for i in range(len(circle_points)):
        circle_point = circle_points[i]
        circle_point_slope = get_slope(points=[circle_center, circle_point])
        dist = np.linalg.norm(circle_point_slope - line_slope)
        if dist < min_dist:
            min_dist = dist
            min_dist_index = i

    return min_dist_index
