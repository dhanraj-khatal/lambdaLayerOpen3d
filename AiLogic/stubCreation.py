from meshlib import mrmeshpy as mm
import pymeshlab
import logging

def stubCreationProcess(inputMeshPath, outputMeshPath):
    try:
        print("stub creation:\ninput:"+inputMeshPath+"\noutput:"+outputMeshPath)
        mesh = mm.loadMesh(inputMeshPath)
        avgEdgeLength = 0.0
        numEdges = 0
        for i in range(mesh.topology.undirectedEdgeSize()):
            dirEdge = mm.EdgeId(i*2)
            org = mesh.topology.org(dirEdge)
            dest = mesh.topology.dest(dirEdge)
            avgEdgeLength += (mesh.points.vec[dest.get()] - mesh.points.vec[org.get()]).length()
            numEdges = numEdges + 1
        avgEdgeLength = avgEdgeLength/numEdges
        fillParams = mm.FillHoleParams()
        fillParams.metric = mm.getUniversalMetric(mesh)
        holes = mesh.topology.findHoleRepresentiveEdges()
        decimateSettings = mm.DecimateSettings()  
        decimateSettings.maxError = 0.25 # some big number not to stop because of this limit
        decimateSettings.packMesh = True
        holesAvailable = False
        for e in holes:
            newFaces = mm.FaceBitSet()
            fillParams.outNewFaces = newFaces
            mm.fillHole(mesh,e,fillParams)
            newVerts = mm.VertBitSet()
            subdivSettings = mm.SubdivideSettings()
            subdivSettings.maxEdgeLen = avgEdgeLength
            subdivSettings.maxEdgeSplits = 20000
            subdivSettings.region = newFaces
            subdivSettings.newVerts = newVerts
            mm.subdivideMesh(mesh,subdivSettings)
            mm.positionVertsSmoothly(mesh, newVerts, mm.LaplacianEdgeWeightsParam.Cotan)
            mm.positionVertsSmoothly(mesh, newVerts, mm.LaplacianEdgeWeightsParam.Cotan)
            decimateSettings.region = newFaces
            decimateSettings.maxDeletedFaces = int(mesh.topology.getValidFaces().count()*0.90) 
            holesAvailable = True  
        if holesAvailable:     
            mm.decimateMesh(mesh, decimateSettings)
        mm.saveMesh(mesh, outputMeshPath)
        return True
    except Exception as e:
        logging.error(e)
        print(e)
        raise e

def createTempBaseProcess(inputMeshPath, outputMeshPath):
    try:
        print("base creation:\ninput:"+inputMeshPath+"\noutput:"+outputMeshPath)
        mesh = mm.loadMesh(inputMeshPath)
        avgEdgeLength = 0.0
        numEdges = 0
        for i in range(mesh.topology.undirectedEdgeSize()):
            dirEdge = mm.EdgeId(i*2)
            org = mesh.topology.org(dirEdge)
            dest = mesh.topology.dest(dirEdge)
            avgEdgeLength += (mesh.points.vec[dest.get()] - mesh.points.vec[org.get()]).length()
            numEdges = numEdges + 1
        avgEdgeLength = avgEdgeLength/numEdges
        fillParams = mm.FillHoleParams()
        fillParams.metric = mm.getUniversalMetric(mesh)
        holes = mesh.topology.findHoleRepresentiveEdges()
        for e in holes:
            newFaces = mm.FaceBitSet()
            fillParams.outNewFaces = newFaces
            mm.fillHole(mesh,e,fillParams)
            newVerts = mm.VertBitSet()
            subdivSettings = mm.SubdivideSettings()
            subdivSettings.maxEdgeLen = avgEdgeLength
            subdivSettings.maxEdgeSplits = 20000
            subdivSettings.region = newFaces
            subdivSettings.newVerts = newVerts
            mm.subdivideMesh(mesh,subdivSettings)

            mm.positionVertsSmoothly(mesh,newVerts)
        mm.saveMesh(mesh, outputMeshPath)
    except Exception as e:
        print("base creation error:\n"+e)
        return False

def createStub(inputMeshPath, outputMeshPath):
    try:
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(inputMeshPath)
        ms.compute_selection_from_mesh_border()
        ms.meshing_close_holes(maxholesize = 300000, selected = True, selfintersection = False, refinehole = True)
        ms.apply_coord_taubin_smoothing()
        ms.apply_selection_erosion()
        ms.compute_coord_by_function(x = 'x + 0.025 * nx', y = 'y + 0.025 * ny', z = 'z + 0.025 * nz', onselected = True)
        ms.apply_selection_dilatation()
        ms.apply_selection_dilatation()
        ms.apply_selection_dilatation()
        ms.apply_coord_taubin_smoothing()
        ms.set_selection_none()
        ms.compute_color_by_function_per_face(r = '175', g = '175', b = '175', a = '0')
        ms.save_current_mesh(outputMeshPath[:-10]+".stl")
        return True
    except Exception as e:
        logging.error(e)
        return False