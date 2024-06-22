import errno
import logging
import os
import traceback

from AiLogic.teethLabeling.utils import save_as_json
from AiLogic.teethLabeling.easyMeshClass import Easy_Mesh


# call this function for API
def get_tooth_boundary_points(
        input_tooth_stl_file: str,
        tooth_label: int,
        output_folder_path: str,
) -> bool:
    """
    For a given labelled tooth, return a list of 12 points
    @param input_tooth_stl_file: path to STL file of a tooth
    @param tooth_label: tooth label of the given input tooth stl file
    @param output_folder_path: path to an already created folder to save the JSON file of required list of points
    @return:
    """
    try:
        if not os.path.isfile(input_tooth_stl_file):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), input_tooth_stl_file)

        if not os.path.isdir(output_folder_path):
            raise NotADirectoryError(errno.ENOENT, os.strerror(errno.ENOENT), output_folder_path)

        tooth_easy_mesh = Easy_Mesh(filename=input_tooth_stl_file)
        tooth_boundary_points: dict = tooth_easy_mesh.get_ordered_boundary_points()

        largest_boundary_points_key = -1
        max_no_of_boundary_points = 0
        for (key, boundary_points) in tooth_boundary_points.items():
            n_boundary_points = len(boundary_points)
            if n_boundary_points > max_no_of_boundary_points:
                max_no_of_boundary_points = n_boundary_points
                largest_boundary_points_key = key

        assert largest_boundary_points_key != -1, \
            'Please pass the tooth stl file without the stubs attached, ' \
            'as the given tooth is not an open mesh'

        save_as_json(
            data=tooth_boundary_points[largest_boundary_points_key],
            file_name=os.path.join(output_folder_path, f'Tooth_{tooth_label}_gum_points.json')
        )

        return True

    except Exception as e:
        logging.error(e)
        tb = traceback.format_exc()
        logging.error(tb)
        return False  
