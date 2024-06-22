import pymeshlab
from AiLogic.createNameStl import getLetterStl
from meshlib import mrmeshpy as mm
import logging

def boolean_operation_engrave(gum, nameStl, outputFile):
    gumMesh = mm.loadMesh(gum)
    nameMesh = mm.loadMesh(nameStl)
    output = mm.boolean(gumMesh, nameMesh, mm.BooleanOperation.DifferenceAB)
    mm.saveMesh(output.mesh, outputFile)

def boolean_operation_emboss(tooth,numberStl,outputFile):
    ms = pymeshlab.MeshSet()
    ms2 = pymeshlab.MeshSet()
    ms.load_new_mesh(tooth)
    ms2.load_new_mesh(numberStl)
    ms.add_mesh(mesh = ms2.current_mesh(), set_as_current = 1)
    # ms.compute_matrix_from_translation_rotation_scale(rotationz = 180)
    ms.generate_boolean_union(first_mesh=0,second_mesh=1)
    ms.save_current_mesh(outputFile)
    ms.clear()
    ms2.clear()
    print("clear mesh maxillary")