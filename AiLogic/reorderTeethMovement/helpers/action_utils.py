import math
from typing import Union, Tuple

import numpy as np
import open3d as o3d
from open3d.cpu.pybind.geometry import LineSet

if __name__ == "action_utils":
    from functions import get_angle_bw_line_and_coaxes
else:
    from .functions import get_angle_bw_line_and_coaxes


def translate_coordinates(
        mesh: Union[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh],
        decimated_mesh: Union[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh, None],
        # line_set: Union[LineSet, None],
        is_dummy: bool = False,
        x_dist: int = 0,
        y_dist: int = 0,
        z_dist: int = 0,
):
    # moving mesh with given distance and angle (x,z,y)
    mesh = mesh.translate([x_dist, y_dist, z_dist])

    if not is_dummy:
        decimated_mesh = decimated_mesh.translate([x_dist, y_dist, z_dist])
        # line_set = line_set.translate([x_dist, y_dist, z_dist])

    return mesh, decimated_mesh
    # return mesh, decimated_mesh, line_set


def translate(
        mesh: Union[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh],
        dist: int = 0,
        angle: int = 0,
):
    d = int(dist)  # distance
    a = int(angle)  # angle

    # direction clock wise
    xx = d * math.cos(math.radians(a))
    yy = d * math.sin(math.radians(a))

    # moving mesh with given distance and angle (x,z,y)
    mesh = mesh.translate([xx, 0, yy])

    return mesh


def translate_tooth(
        mesh: Union[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh],
        mesh_box: Union[o3d.geometry.PointCloud, o3d.geometry.TriangleMesh, None],
        line_set: Union[LineSet, None],
        dist: int = 0,
        angle: int = 0,
        ie_dist: int = 0,
        root_axis_index: int = 0,
):
    # direction clock wise
    x_dist = round(dist * math.cos(math.radians(angle)), 10)
    y_dist = round(dist * math.sin(math.radians(angle)), 10)

    x_ie_dist, y_ie_dist, z_ie_dist = 0, 0, 0
    if line_set is not None:
        x_ie_dist, y_ie_dist, z_ie_dist = intrude_extrude_tooth(line_set, ie_dist, root_axis_index)
        x_dist += round(x_ie_dist, 10)
        y_dist += round(y_ie_dist, 10)
        z_ie_dist = round(z_ie_dist, 10)
        line_set = line_set.translate([x_dist, y_dist, z_ie_dist])

    # moving mesh with given distance and angle (x,z,y)
    mesh = mesh.translate([x_dist, y_dist, z_ie_dist])
    if mesh_box is not None:
        mesh_box = mesh_box.translate([x_dist, y_dist, z_ie_dist])

    print(x_dist, end="")
    print(",", y_dist, end="")
    print(",", z_ie_dist, end="")
    print()

    return mesh, mesh_box, line_set


def get_rotation_matrix(
        angle: float = 0.0,
        slope: np.array = None
):
    # calculating distance
    d = np.sqrt(np.sum(slope ** 2))
    # calculating slope
    q1 = slope / d * math.sin(math.radians(angle))
    q0 = math.cos(math.radians(angle))

    quaternion = np.insert(q1, 0, q0)
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(quaternion)

    return rotation_matrix


def scale(
        val: float,
        src_scale: Tuple = (-1, 1),
        dst_scale: Tuple = (0, 20)
):
    """
    if using continuous actions, rescale
    Scale the given value from the scale of src to the scale of dst.
    """

    scale_ratio = (val - src_scale[0]) / (src_scale[1] - src_scale[0])
    return scale_ratio * (dst_scale[1] - dst_scale[0]) + dst_scale[0]


def intrude_extrude_tooth(
        line_set: LineSet,
        dist: int = 0,
        root_axis_index: int = 0,
):
    # root axis points
    points = line_set.get_line_coordinate(root_axis_index)
    slope, x_angle, y_angle, z_angle = get_angle_bw_line_and_coaxes(points)
    x_dist = dist * math.cos(math.radians(x_angle))
    y_dist = dist * math.cos(math.radians(y_angle))
    z_dist = dist * math.cos(math.radians(z_angle))

    return x_dist, y_dist, z_dist
