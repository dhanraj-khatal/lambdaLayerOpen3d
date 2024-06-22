import errno
import logging
import os
import shutil
import time
import traceback

import numpy as np
import open3d as o3d
from joblib import Parallel, delayed

from AiLogic.teethLabeling.utils import get_json_object, get_slope, save_as_json, swap_z_and_y_coordinates, \
    convert_point_to_3d_team_format

EPSILON = 0.0001
SIDE_AXIS_LENGTH = 7
INCISORS_RANGE = list(range(6, 12)) + list(range(22, 28))

_logger = logging.getLogger('saveToothAxes')


# API function - Call this function from outside
def save_teeth_axes(
        case_folder: str,
        points_folder: str,
        boundary_points_folder: str,
        output_folder: str,
) -> bool:
    """
    @param case_folder: folder path containing the Tooth_i_clean.stl files, present in the folder named as `preCloseMeshNonManifold`

    @param points_folder: folder path containing the Tooth_i_points.json files,
    returned from the points tool

    @param boundary_points_folder: folder containing the Tooth_i_gum_points.json files for all the teeth, obtained from the boundary points API

    @param output_folder: folder path to save the json files per tooth containing
    the axes details
    """

    try:
        st = time.time()

        if not os.path.isdir(case_folder):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), case_folder)

        if not os.path.isdir(points_folder):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), points_folder)

        if not os.path.isdir(boundary_points_folder):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), boundary_points_folder)

        # delete temp and out folders, if already present
        if os.path.isdir(output_folder):
            shutil.rmtree(output_folder)

        # create temp and out folders
        os.mkdir(output_folder)

        Parallel(n_jobs=4)(
            delayed(save_tooth_axis)(
                case_folder,
                points_folder,
                boundary_points_folder,
                output_folder,
                tooth_id=i + 1
            )
            for i in range(32))

        et = time.time()
        _logger.info(f'Saved root axis json files in time {et - st}')

        return True

    except Exception as e:
        logging.error(e)
        tb = traceback.format_exc()
        logging.error(tb)
        return False


# HELPER function - Do not call this function from outside
def save_tooth_axis(
        case_folder: str,
        points_folder: str,
        boundary_points_folder: str,
        output_folder: str,
        tooth_id: int,
):
    """
    Save the default axis information of each of the tooth:
    - top_axis: axis from the crown center to the top point on the crown,
    - front_axis: axis from the crown center to the front point on the crown,
    - side_axis: axis from the crown center to the side point on the crown

    @param case_folder: folder containing STL files for all the teeth
    @param points_folder: folder containing the point JSON files for all the teeth
    @param boundary_points_folder: folder containing the boundary points JSON files for all the teeth,
        obtained from the boundary points API
    @param output_folder: folder which will contain below outputs:
        - 'Tooth_i_axes.json' JSON file per tooth STL containing the axis information
    @param tooth_id: tooth id (1-based) of the current tooth
    @return:
    """
    tooth_crown_filename = os.path.join(case_folder, f'Tooth_{tooth_id}_clean.stl')
    point_json_filename = os.path.join(points_folder, f'Tooth_{tooth_id}_points.json')
    boundary_points_json_filename = os.path.join(boundary_points_folder, f'Tooth_{tooth_id}_gum_points.json')

    if not os.path.isfile(tooth_crown_filename) or \
            not os.path.isfile(point_json_filename) or \
            not os.path.isfile(boundary_points_json_filename):
        return

    o3d_mesh = o3d.io.read_triangle_mesh(tooth_crown_filename)
    crown_center = o3d_mesh.get_center()

    points_picked = get_json_object(point_json_filename)
    boundary_points = np.asarray(get_json_object(boundary_points_json_filename))
    bone_center = np.mean(boundary_points, axis=0)

    crown_left_point = swap_z_and_y_coordinates(points_picked["top_point1"])
    crown_right_point = swap_z_and_y_coordinates(points_picked["top_point2"])
    top_point = (np.asarray(crown_left_point) + np.asarray(crown_right_point)) / 2

    front_point = swap_z_and_y_coordinates(points_picked["front_point"])

    axes_center = bone_center
    if tooth_id in INCISORS_RANGE:
        axes_center = crown_center

    top_axis_slope = get_slope([axes_center, top_point])
    front_axis_slope = get_slope([axes_center, front_point])
    side_axis_slope = np.cross(front_axis_slope, top_axis_slope)
    new_front_axis_slope = np.cross(top_axis_slope, side_axis_slope)

    top_point = axes_center + (SIDE_AXIS_LENGTH * top_axis_slope)
    front_point = axes_center + (SIDE_AXIS_LENGTH * new_front_axis_slope)
    side_point = axes_center + (SIDE_AXIS_LENGTH * side_axis_slope)

    data = {
        "tooth_crown_center": convert_point_to_3d_team_format(axes_center),
        "bone_center": convert_point_to_3d_team_format(bone_center),

        "top_point": convert_point_to_3d_team_format(top_point),
        "front_point": convert_point_to_3d_team_format(front_point),
        "side_point": convert_point_to_3d_team_format(side_point),

        "top_axis": convert_point_to_3d_team_format(top_axis_slope),
        "front_axis": convert_point_to_3d_team_format(new_front_axis_slope),
        "side_axis": convert_point_to_3d_team_format(side_axis_slope),
    }

    tooth_axes_filename = os.path.join(output_folder, f'Tooth_{tooth_id}_axes.json')
    save_as_json(data, tooth_axes_filename)
    _logger.info(f'Saved axes details at {tooth_axes_filename}')
