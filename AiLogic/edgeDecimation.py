import pymeshlab
import logging

def edgeDecimationProcess(inputMeshPath, outputMeshPath):
    try:
        print("edge decimation:\ninput:"+inputMeshPath+"\noutput:"+outputMeshPath)
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(inputMeshPath)
        ms.compute_selection_from_mesh_border()
        ms.meshing_decimation_quadric_edge_collapse(autoclean=True, selected=True)
        ms.meshing_decimation_quadric_edge_collapse(autoclean=True, selected=True)
        ms.save_current_mesh(outputMeshPath)
        return True
    except Exception as e:
        logging.error(e)
        print("edge decimation error:\n")
        return False