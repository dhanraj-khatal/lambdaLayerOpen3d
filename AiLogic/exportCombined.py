import os
import sys
import errno
import time
import vedo



def export_combined(input_gum_path: str,input_teeth_path: str):

    """
    @params:
    input_gum_path: path to the stl file, work for both max and mand

    returns: 
    new mesh with name {input_gum_path}_decimated.stl 
    """

    # simple IO error checks
    if not os.path.isfile(input_gum_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), input_gum_path)
    st = time.time()
    output_filename = str(input_gum_path[:-4]) + '_decimated.stl'

    mesh = vedo.load(input_gum_path)

    target_num = 80000

    if target_num <= mesh.NCells():
        ratio = target_num / mesh.NCells()  # calculate ratio
        mesh.decimate(fraction=ratio)
    
    #write new mesh irrespective 
    vedo.write(mesh, output_filename)
    et = time.time()
    print(f'Successfully completed function call in time {et - st}')
