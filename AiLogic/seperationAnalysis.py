import open3d as o3d
import pymeshlab
import json
import logging
from flask import jsonify
import utils.plyToGlbConversion as ply_to_glb

# input_mesh : file name to load in mesh


def sep_curve(input_mesh):
    try:
        ms = pymeshlab.MeshSet()
        ms1 = pymeshlab.MeshSet()

        # load a new mesh in the MeshSet, and sets it as current mesh
        # the path of the mesh can be absolute or relative
        ms.load_new_mesh(input_mesh)
        logging.info(len(ms))  # now ms contains 1 mesh
        # instead of len(ms) you can also use:
        logging.info(ms.number_meshes())
        ms.compute_curvature_principal_directions_per_vertex(
            method=3, curvcolormethod=3)
        ms.compute_selection_from_mesh_border()
        ms.apply_selection_dilatation()
        ms.apply_selection_dilatation()
        ms.apply_selection_dilatation()
        ms.set_color_per_vertex(
            color1=pymeshlab.Color(0, 0, 255, 255), onselected=1)

        ms.compute_selection_by_condition_per_vertex(condselect="r>230")
        ms.apply_selection_inverse(invverts=1)
        ms.meshing_remove_selected_vertices()
        ms.set_color_per_vertex(color1=pymeshlab.Color(255, 0, 0, 255))
        # ms.save_current_mesh('only_curve.ply')
        ms.save_current_mesh(input_mesh[:-4]+'_only_curve.stl')
        ms1.load_new_mesh(input_mesh)
        ms.add_mesh(mesh=ms1.current_mesh(), set_as_current=1)
        # per = pmeshlab.Percentage.set_value(0.001)
        ms.transfer_attributes_per_vertex(upperbound=pymeshlab.Percentage(
            0.001), colortransfer=1, sourcemesh=0, targetmesh=1)
        # ms.apply_coord_laplacian_smoothing(stepsmoothnum=100)
        ply_filename = input_mesh[:-4]+"_Red_Vertex_Color_Cleaned.ply"
        glb_filename = input_mesh[:-4]+"_Red_Vertex_Color_Cleaned.glb"
        ms.save_current_mesh(ply_filename)
        ply_to_glb.convert_from_ply_to_glb(ply_filename, glb_filename)

        # upperjaw = o3d.io.read_triangle_mesh(
        #     input_mesh[:-4]+"_Red_Vertex_Color_Cleaned.ply")        
        # o3d.io.write_triangle_mesh(
        #     input_mesh[:-4]+"_Red_Vertex_Color_Cleaned.glb", upperjaw)
    except Exception as e:
        logging.error(e)
        return jsonify({'result': 'Error'})
