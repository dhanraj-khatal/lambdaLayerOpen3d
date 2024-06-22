import pymeshlab
# import open3d as o3d
# import numpy as np
# from scipy.spatial.transform import Rotation as R
from flask import jsonify

def tooth_fill_hole_py(input_mesh, fileNameForCol):
    try:
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(input_mesh)
        ms.compute_selection_from_mesh_border()
        ms.meshing_decimation_quadric_edge_collapse(
            autoclean=True, selected=True)
        ms.apply_selection_dilatation()
        ms.apply_coord_hc_laplacian_smoothing()
        ms.apply_coord_hc_laplacian_smoothing()
        ms.meshing_close_holes(
            maxholesize=100000, newfaceselected=True, selfintersection=False)
        ms.meshing_surface_subdivision_ls3_loop(
            threshold=pymeshlab.Percentage(1.000), selected=True)
        ms.save_current_mesh(input_mesh[:-10]+".stl")
        ms.meshing_remove_selected_faces()
        ms.meshing_close_holes(maxholesize=500, selfintersection=False)
        ms.save_current_mesh(fileNameForCol[:-10]+".stl")
        return True
    except Exception as e:
        print(e)
        return False

