import pymeshlab

def maxillary_slicing_operation(gum, plane, gum_base, gumSlicingFile):
    ms = pymeshlab.MeshSet()
    ms2 = pymeshlab.MeshSet()
    ms3 = pymeshlab.MeshSet()

    ms.load_new_mesh(gum)
    ms2.load_new_mesh(plane)
    ms3.load_new_mesh(gum_base)

    

    # difference
    ms.add_mesh(mesh=ms2.current_mesh(), set_as_current=1)
    ms.meshing_invert_face_orientation(forceflip=True, onlyselected=False)
    ms.apply_matrix_flip_or_swap_axis(flipx=True)
    ms.meshing_invert_face_orientation(forceflip=True, onlyselected=False)
    ms.generate_boolean_difference(first_mesh=0, second_mesh=1)
    ms2.apply_matrix_flip_or_swap_axis(flipz=True)

    # union
    ms.add_mesh(mesh=ms3.current_mesh(), set_as_current=3)
    ms.generate_boolean_union(first_mesh=2, second_mesh=3)
    ms.apply_coord_hc_laplacian_smoothing()
    ms.apply_coord_hc_laplacian_smoothing()
    ms.apply_coord_hc_laplacian_smoothing()
    ms.apply_coord_hc_laplacian_smoothing()

    # ms.save_current_mesh(gumSlicingFile)
    ms.save_current_mesh(gumSlicingFile)
