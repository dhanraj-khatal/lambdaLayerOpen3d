import os
import errno
import argparse
import trimesh


def convert_from_ply_to_glb(ply_filename: str, glb_filename: str):
    """
    :param ply_filename: Path of the input ply file with extension
    :param glb_filename: Output path required for the glb/gltf file with extension
    :return: Nothing. Saves the glb mesh to the output path
    """
    mesh = trimesh.load(ply_filename)
    scene = trimesh.Scene([mesh])
    _ = scene.export(file_obj=glb_filename)