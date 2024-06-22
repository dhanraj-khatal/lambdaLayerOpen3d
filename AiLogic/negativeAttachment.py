from meshlib import mrmeshpy as mm
import logging

def negativeAttachment(tooth,attachment,outputFile):
    logging.info("negative attachment operation started\n")
    toothMesh = mm.loadMesh(tooth)
    attachmentMesh = mm.loadMesh(attachment)
    output = mm.boolean(toothMesh, attachmentMesh, mm.BooleanOperation.DifferenceAB)
    mm.saveMesh(output.mesh, outputFile)
    logging.info("negative attachment operation operation end\n")