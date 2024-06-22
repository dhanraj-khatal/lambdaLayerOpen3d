import argparse
import errno
import os
import shutil
import time
import pickle

import numpy as np

from AiLogic.createCircles.alignedTeethMesh import AlignedTeethMesh
from AiLogic.createCircles.utils import save_as_json, get_json_object

def create_new_circle(
        aligned_teeth_mesh: str,
        circle_points_json: str,
        out: str,
        out_file_name: str
):
    """
    @params:
    aligned_teeth_mesh: path to the pkl file of aligned teeth mesh obtained from the gvt output,
    circle_points_json: path to the json file containing the points selected by the user on the mesh,
    out: output folder path,
    out_file_name: output file name for the new circle json file
    """

    # simple IO error checks
    if not os.path.isfile(aligned_teeth_mesh):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), aligned_teeth_mesh)

    # delete out folder, if already present
    if os.path.isdir(out):
        shutil.rmtree(out)

    # create out folders
    os.mkdir(out)

    st = time.time()

    f = open(aligned_teeth_mesh, 'rb')
    point_ids = pickle.load(f)
    mesh_with_gums = AlignedTeethMesh(point_ids)

    new_circle_points_object = get_json_object(filepath=circle_points_json)
    new_circle_points = []
    new_circle_point_ids = []
    # assuming the maximum no of points drawn by user is 200:
    for i in range(1, 200):
        key = f'top_point_{i}'
        if key not in new_circle_points_object.keys():
            continue

        point_data = new_circle_points_object[key]
        point = np.asarray([point_data["_x"], point_data["_z"], point_data["_y"]])
        new_circle_points.append(point)
        new_circle_point_ids.append(mesh_with_gums.find_closest_point(point))

    new_circle_point_ids.append(new_circle_point_ids[0])

    circular_points = []
    for i in range(len(new_circle_point_ids) - 1):
        joining_path = mesh_with_gums.find_astar_curvature_path(
            source_point_id=new_circle_point_ids[i + 1],
            sink_point_id=new_circle_point_ids[i],
        )
        for j in range(len(joining_path) - 1):
            circular_points.append(mesh_with_gums.get_point(joining_path[j]))

    save_as_json(circular_points, os.path.join(out, f'{out_file_name}.json'))

    et = time.time()
    print(f'Circle created for {aligned_teeth_mesh} with file name: {out_file_name}')
    print(f'Successfully completed function call in time {et - st}')
