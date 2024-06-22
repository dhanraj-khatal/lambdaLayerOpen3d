import pymeshlab
import open3d as o3d
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
import logging


def actual_separation(input_mesh, stl_file):
    try:
        logging.info("actual_separation - START")
        logging.info("actual_separation - input_mesh" + input_mesh)

        triangle_mesh = o3d.io.read_triangle_mesh(input_mesh)
        
        logging.info("actual_separation - read triangle mesh")

        r = R.from_euler('x', 90, degrees=True)
        logging.info("actual_separation - R from euler")
        R1 = r.as_matrix()
        logging.info("actual_separation - R1")
        # triangle_mesh.rotate(R1, center=(0, 0, 0))
        o3d.io.write_triangle_mesh(input_mesh[:-4]+".ply", triangle_mesh)
        logging.info("actual_separation - Write triangle mesh")

        time.sleep(6)
        logging.info("Sleep of 6 sec complete")

        input_mesh = input_mesh[:-4]+".ply"
        logging.info("New mesh is PLY file")

        ms = pymeshlab.MeshSet()
        ms1 = pymeshlab.MeshSet()
        ms2 = pymeshlab.MeshSet()

        logging.info("init ms, ms1, ms2")


        # load a new mesh in the MeshSet, and sets it as current mesh
        # the path of the mesh can be absolute or relative
        """ms.load_new_mesh(input_mesh)
        print(len(ms))  # now ms contains 1 mesh
        # instead of len(ms) you can also use:
        print(ms.number_meshes())
        ms.compute_curvature_principal_directions_per_vertex(method=3, curvcolormethod=3)
        ms.compute_selection_from_mesh_border()
        ms.apply_selection_dilatation()
        ms.apply_selection_dilatation()
        ms.apply_selection_dilatation()
        ms.set_color_per_vertex(color1= pymeshlab.Color(0,0,255,255), onselected =1)
        
        ms.compute_selection_by_condition_per_vertex(condselect = "r>230")
        ms.apply_selection_inverse(invverts=1)
        ms.meshing_remove_selected_vertices()
        ms.set_color_per_vertex(color1= pymeshlab.Color(255,0,0,255))
        ms.save_current_mesh(input_mesh[:-4]+'_only_curve.stl')"""

        logging.info("Load new mesh PLY")

        ms.load_new_mesh(input_mesh)
        ms.compute_selection_by_condition_per_vertex(condselect="g==255")
        ms.meshing_remove_selected_vertices()
        logging.info("Save as separated stl")
        ms.save_current_mesh(input_mesh[:-4]+"_Separated.stl")
        ms1.load_new_mesh(input_mesh[:-4]+"_Separated.stl")
        ms1.meshing_remove_connected_component_by_face_number(mincomponentsize=700)
        ms1.set_color_per_vertex(color1=pymeshlab.Color(255, 0, 0, 255))
        ms2.load_new_mesh(stl_file)
        ms1.add_mesh(mesh=ms2.current_mesh(), set_as_current=1)
        ms1.transfer_attributes_per_vertex(upperbound=pymeshlab.Percentage(
            0.01), colortransfer=1, sourcemesh=0, targetmesh=1)
        ms1.compute_selection_by_condition_per_vertex(condselect="g==0")
        ms1.meshing_remove_selected_vertices()
        #ms1.save_current_mesh("step1.stl")
        ms1.compute_color_by_conntected_component_per_face()
        ms1.compute_color_transfer_face_to_vertex()
        ms1.meshing_remove_connected_component_by_face_number(
            mincomponentsize=50, removeunref=1)
        ms1.compute_selection_from_mesh_border()
        ms1.apply_selection_dilatation()
        ms1.apply_coord_laplacian_smoothing_surface_preserving(
            selection=1, angledeg=5, iterations=30)
        #ms1.transfer_attributes_per_vertex()

        ms1.save_current_mesh(input_mesh[:-4]+"_final.ply")
        triangle_mesh = o3d.io.read_triangle_mesh(input_mesh[:-4]+"_final.ply")
        r = R.from_euler('x', 90, degrees=True)
        R1 = r.as_matrix()
        triangle_mesh.rotate(R1, center=(0, 0, 0))
        o3d.io.write_triangle_mesh(input_mesh[:-4]+"_final.glb", triangle_mesh)
        r = R.from_euler('x', -90, degrees=True)
        R1 = r.as_matrix()
        triangle_mesh.rotate(R1, center=(0, 0, 0))
        triangle_mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(
            triangle_mesh)
        o3d.io.write_triangle_mesh(input_mesh[:-4]+"_final.stl", triangle_mesh)
        """ms.add_mesh(mesh = ms1.current_mesh(),set_as_current = 1)
        #per = pmeshlab.Percentage.set_value(0.001)
        ms.transfer_attributes_per_vertex(upperbound = pymeshlab.Percentage(0.001), colortransfer =1, sourcemesh =0, targetmesh=1)
        #ms.apply_coord_laplacian_smoothing(stepsmoothnum=100)
        ms.save_current_mesh(input_mesh[:-4]+"_with_sep_curve.ply")"""
        logging.info("actual_separation - END")
    except Exception as e:
        logging.info('Error occurred in actual_separation')
        logging.error(e)