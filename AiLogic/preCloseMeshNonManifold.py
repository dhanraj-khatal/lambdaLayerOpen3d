from pickle import TRUE
import pymeshlab
import open3d as o3d
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from meshlib import mrmeshpy as mm
import logging

#input_mesh = "99752562_shell_occlusion_u.stl"
def clean_mesh(input_mesh):

    try:
        ms = pymeshlab.MeshSet()
        ms1 = pymeshlab.MeshSet()
        ms2 = pymeshlab.MeshSet()
        
        ms.load_new_mesh(input_mesh)
        ms.compute_selection_by_non_manifold_edges_per_face()
        ms.meshing_remove_selected_vertices_and_faces()
        ms.compute_selection_by_non_manifold_per_vertex()
        ms.meshing_remove_selected_vertices()
        ms.meshing_remove_connected_component_by_face_number(mincomponentsize = 50, removeunref =1)
        ms.set_selection_none()
        
        ms.save_current_mesh(input_mesh[:-4]+"_clean.stl")
        return True
    except Exception as e:
        print(e)
        return False

def healMeshProcess(inputMesh, decimate=True):
    try:
        mesh = mm.loadMesh(inputMesh)
        voxelSize = mesh.computeBoundingBox().diagonal() * 5e-3
        print(f'voxel size: {voxelSize}')
        numHoles = mm.findRightBoundary( mesh.topology ).size()
        oParams = mm.GeneralOffsetParameters()
        if (numHoles != 0):
            oParams.signDetectionMode = mm.SignDetectionMode.HoleWindingRule
        oParams.voxelSize = voxelSize
        resMesh = mm.generalOffsetMesh( mesh, 0.0, oParams)
        if(decimate):
            resMesh.packOptimally(False)
            dSettings = mm.DecimateSettings()
            dSettings.maxError = 0.25 * voxelSize
            dSettings.tinyEdgeLength = mesh.computeBoundingBox().diagonal() * 1e-4
            dSettings.stabilizer = 1e-5
            dSettings.packMesh = True
            dSettings.subdivideParts = 64
            mm.decimateMesh(resMesh,dSettings)
        mm.saveMesh(resMesh, inputMesh[:-4]+"_clean.stl")
        return True
    except Exception as e:
        print(e)
        return False
