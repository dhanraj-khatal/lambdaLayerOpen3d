import pymeshlab
from meshlib import mrmeshpy as mm

def cutting_operation(gum, plane,base_gum, outputFile):
    try:
        ms = pymeshlab.MeshSet()
        ms2 = pymeshlab.MeshSet()
        ms3 = pymeshlab.MeshSet()

        try:
            ms.load_new_mesh(gum)
            ms2.load_new_mesh(plane)
            ms3.load_new_mesh(base_gum)
        except Exception as e:
            ms.load_new_mesh(gum)
            ms2.load_new_mesh(plane)
            ms3.load_new_mesh(base_gum)
        
        
        #invert_face_orinetation
        ms.meshing_invert_face_orientation(forceflip = True, onlyselected = False)
        ms2.meshing_invert_face_orientation(forceflip = True, onlyselected = False)

        #difference
        ms.add_mesh(mesh = ms2.current_mesh(), set_as_current = 1)
        ms.generate_boolean_difference(first_mesh=0,second_mesh=1)
        
        
        #union
        ms3.meshing_invert_face_orientation(forceflip = True, onlyselected = False)
        ms.add_mesh(mesh = ms3.current_mesh(), set_as_current = 3)
        ms.generate_boolean_union(first_mesh=2,second_mesh=3)
        ms.apply_coord_taubin_smoothing(stepsmoothnum = 50)
        
        ms.save_current_mesh(outputFile)  
        print("clear mesh data gum cutting")
    except Exception as e:
        raise e

def gumCuttingProcess(gum, cuttingPlane, gumBase, outputFile):
    gumMesh = mm.loadMesh(gum)
    cuttingPlaneMesh = mm.loadMesh(cuttingPlane)
    gumBaseMesh = mm.loadMesh(gumBase)
    gumMesh.topology.flipOrientation()
    cuttingPlaneMesh.topology.flipOrientation()
    gumBaseMesh.topology.flipOrientation()
    voxelSize = gumMesh.computeBoundingBox().diagonal() * 5e-3  # offset grid precision (algorithm is voxel based)
    print(f'voxel size: {voxelSize}')
    gumCutDifferenceOutput = mm.boolean(cuttingPlaneMesh, gumMesh, mm.BooleanOperation.DifferenceBA)
    healMesh = healProcess(gumCutDifferenceOutput.mesh, voxelSize)
    gumBaseUnionOutput = mm.boolean(gumBaseMesh, healMesh, mm.BooleanOperation.Union)
    mm.saveMesh(gumBaseUnionOutput.mesh, outputFile)

def healProcess(mesh, voxelSize, decimate=True):
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
    return resMesh