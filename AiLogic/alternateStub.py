import pymeshlab

def alternateStub(input_mesh,outputFile):
    try:
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(input_mesh)
        ms.compute_selection_from_mesh_border()
        ms.meshing_close_holes(maxholesize=100000, selected = True,newfaceselected = True, selfintersection = False)
        ms.meshing_surface_subdivision_ls3_loop(threshold = pymeshlab.Percentage(0.840),selected = True)
        ms.save_current_mesh(outputFile)
        return True
    except Exception as e:
        print(e)
        return False