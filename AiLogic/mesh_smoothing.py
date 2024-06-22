import pymeshlab

def smoothing_operation(gum,outpuFile):
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(gum)
    ms.add_mesh(mesh = ms.current_mesh(), set_as_current = 1)
    ms.apply_coord_hc_laplacian_smoothing()
    ms.apply_coord_hc_laplacian_smoothing()
    ms.apply_coord_hc_laplacian_smoothing()
    ms.apply_coord_hc_laplacian_smoothing()
    ms.apply_matrix_flip_or_swap_axis(flipx = True)
    ms.meshing_invert_face_orientation()
    ms.save_current_mesh(outpuFile)
