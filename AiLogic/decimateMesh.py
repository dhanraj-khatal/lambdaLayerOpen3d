import os
import errno
import time

import pymeshlab
import vedo


def save_clean_mesh(input_filename, output_filename):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(input_filename)
    ms.compute_selection_by_non_manifold_edges_per_face()
    ms.meshing_remove_selected_vertices_and_faces()
    ms.compute_selection_by_non_manifold_per_vertex()
    ms.meshing_remove_selected_vertices()
    ms.meshing_remove_connected_component_by_face_number(mincomponentsize=50, removeunref=1)

    ms.save_current_mesh(output_filename)


def decimate_mesh(input_mesh_path: str):
    """
    @params:
    input_mesh_path: path to the stl file, work for both max and mand

    returns: 
    new mesh with name {input_mesh_path}_decimated.stl 
    """

    # simple IO error checks
    if not os.path.isfile(input_mesh_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), input_mesh_path)

    st = time.time()

    mesh = vedo.load(input_mesh_path)

    target_num = 160000

    if target_num <= mesh.NCells():
        ratio = target_num / mesh.NCells()  # calculate ratio
        mesh.decimate(fraction=ratio)

    # write new mesh irrespective
    temp_output_filename = str(input_mesh_path[:-4]) + '_decimated.stl'
    output_filename = str(input_mesh_path[:-4]) + '_decimated.stl'

    vedo.write(mesh, temp_output_filename)
    save_clean_mesh(
        input_filename=temp_output_filename,
        output_filename=output_filename
    )

    et = time.time()
    print(f'Successfully completed function call in time {et - st}')
