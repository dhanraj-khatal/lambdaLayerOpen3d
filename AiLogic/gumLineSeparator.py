import pymeshlab
from meshlib import mrmeshpy as mm
import os
import logging

def heal( mesh : mm.Mesh, voxelSize : float, decimate : bool = True )->mm.Mesh:
	numHoles = mm.findRightBoundary( mesh.topology ).size()
	oParams = mm.GeneralOffsetParameters()
	if (numHoles != 0):
		oParams.signDetectionMode = mm.SignDetectionMode.HoleWindingRule
	oParams.voxelSize = voxelSize
	resMesh = mm.generalOffsetMesh( mesh, 0.0, oParams)
	if ( decimate ):
		resMesh.packOptimally(False)
		dSettings = mm.DecimateSettings()
		dSettings.maxError = 0.25 * voxelSize
		dSettings.tinyEdgeLength = mesh.computeBoundingBox().diagonal() * 1e-4
		dSettings.stabilizer = 1e-5
		dSettings.packMesh = True
		dSettings.subdivideParts = 64
		mm.decimateMesh(resMesh,dSettings)
	return resMesh

def jawAndCurveBooleanOperation(jawFilePath, curveFilePath, outputFilePath):
    logging.info("boolean operation started\n")
    jawMesh = mm.loadMesh(jawFilePath)
    curveMesh = mm.loadMesh(curveFilePath)
    voxelSize = mm.suggestVoxelSize( curveMesh, 6e7 )
    logging.info(f'voxel size: {voxelSize}')
    healCurve = heal(curveMesh, voxelSize)
    output = mm.boolean(healCurve, jawMesh, mm.BooleanOperation.OutsideB)
    mm.saveMesh(output.mesh, outputFilePath)
    logging.info("boolean operation end\n")

def gumVsTeethSegmentation(differenceMeshFile, outputFolderPath):
    logging.info("segmentation started\n")
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(differenceMeshFile)
    logging.info("mesh loaded\n")
    ms.meshing_remove_connected_component_by_face_number(mincomponentsize = 1800, removeunref = True)
    fileNameForOutputSegmentation = os.path.join(outputFolderPath, 'segmentation.stl')
    logging.info("meshing complete\n")
    ms.save_current_mesh(fileNameForOutputSegmentation)
    ms.compute_selection_by_small_disconnected_components_per_face(nbfaceratio = 1, nbneighbors = 16, nonclosedonly = False)
    ms.apply_selection_inverse(invfaces = True, invverts = False)
    ms.generate_from_selected_faces(deleteoriginal = True)
    fileNameForOutputGumSegment = os.path.join(outputFolderPath, 'gum.stl')
    ms.save_current_mesh(fileNameForOutputGumSegment)
    ms.delete_current_mesh()
    # ms.compute_curvature_principal_directions_per_vertex(curvcolormethod = 3, scale = pymeshlab.PercentageValue(0.999997))
    # logging.info("color curv vertex complete\n")
    fileNameForOutputTeethSegment = os.path.join(outputFolderPath, 'teeth.ply')
    ms.save_current_mesh(fileNameForOutputTeethSegment)
    ms.generate_splitting_by_connected_components(delete_source_mesh=True)
    numberOfMeshes = ms.number_meshes()
    logging.info(f'number of teeth: {numberOfMeshes}\n')
    for meshNumber in range(numberOfMeshes+2):
        try:
            ms.set_current_mesh(new_curr_id = meshNumber)
            outputColorFileName = os.path.join(outputFolderPath, "Tooth_"+str(meshNumber)+".ply")
            outputFileName = os.path.join(outputFolderPath, "Tooth_"+str(meshNumber)+".stl")
            ms.save_current_mesh(outputFileName)
            ms.save_current_mesh(outputColorFileName)
            logging.info(f'tooth {meshNumber} saved\n')
        except Exception as e:
            print(e)
            logging.info(e)
    logging.info("segmentation complete\n")

def teethSegmentation(teethFilePath, availableFilePath):
    try:
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(teethFilePath)
        ms.compute_curvature_principal_directions_per_vertex(curvcolormethod = 3, scale = pymeshlab.PercentageValue(1.000007))
        ms.compute_selection_by_color_per_face(color = pymeshlab.Color(255, 0, 0, 255))
        ms.generate_from_selected_faces()
        
        ms.set_current_mesh(new_curr_id = 1)
        ms.compute_selection_by_small_disconnected_components_per_face()
        ms.generate_from_selected_faces()

        ms.set_current_mesh(new_curr_id = 1)
        ms.delete_current_mesh()

        ms.generate_by_merging_visible_meshes()
        ms.compute_selection_from_mesh_border()
        ms.apply_selection_dilatation()
        ms.apply_coord_hc_laplacian_smoothing()
        ms.meshing_remove_connected_component_by_face_number(mincomponentsize = 100, removeunref = True)
        ms.generate_splitting_by_connected_components(delete_source_mesh=True)
        numberOfMeshes = ms.number_meshes()
        print(f"Number of meshes: {numberOfMeshes}")
        logging.info(f'number of teeth: {numberOfMeshes}\n')
        for i in range(4, numberOfMeshes+4):
            print(f"Processing mesh {i}: {ms.mesh_id_exists(id = i)}")
            logging.info(f'Processing mesh {i}: {ms.mesh_id_exists(id = i)}\n')
            if(ms.mesh_id_exists(id = i)):
                ms.set_current_mesh(new_curr_id = i)
                ms.save_current_mesh(availableFilePath[i-4])
    except Exception as e:
        print(e)
        logging.info(e)
        raise e