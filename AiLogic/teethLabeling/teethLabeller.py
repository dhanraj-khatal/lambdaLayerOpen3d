import os
import numpy as np
import open3d as o3d
import logging

from AiLogic.teethLabeling.utils import get_json_object, find_leftmost_and_rightmost_indices


logger = logging.getLogger('teethLabeller')
logger.setLevel(logging.INFO)


def teeth_labelling(
        jaw_type: str,
        labels_json: str,
        unlabelled_stls_folder_path: str,
        output_folder_path: str,
):
    """
    Label the teeth in the unlabelled stls folder.

    @param jaw_type: type of jaw (mandibular/maxillary)
    @param labels_json: json file containing labelled points
    @param unlabelled_stls_folder_path: folder containing unlabelled stl files
    @param output_folder_path: folder to save labelled stl files
    """

    if jaw_type not in ['mandibular', 'maxillary']:
        raise ValueError("Jaw type must be either 'mandibular' or 'maxillary'")

    teeth_labels = get_json_object(labels_json)[jaw_type]
    if jaw_type == 'mandibular':
        teeth_labels = teeth_labels[::-1]

    logger.info(f"Teeth labels: {teeth_labels}")

    tooth_objects = []
    tooth_centers = []
    for tooth_id in range(1, 18):
        filepath = os.path.join(unlabelled_stls_folder_path, f'Tooth_{tooth_id}.stl')
        if not os.path.isfile(filepath):
            continue

        tooth: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(filepath)
        tooth.compute_vertex_normals()

        tooth_objects.append(tooth)
        tooth_centers.append(tooth.get_center())

    avg_center = np.mean(tooth_centers, axis=0)

    left_i, right_i = find_leftmost_and_rightmost_indices(
        points_list=tooth_centers,
        center_point=avg_center,
    )

    logger.info(f"Leftmost tooth stl: {left_i+1}, Rightmost tooth stl: {right_i+1}")

    # finding neighboring teeth of the leftmost tooth
    labelled_teeth = [left_i]

    tooth_idx = left_i
    counter = 1
    while counter < len(teeth_labels) - 1:
        closest_tooth_idx = None
        closest_tooth_dist = 1000000
        for i, tooth in enumerate(tooth_objects):
            if i == tooth_idx or i in labelled_teeth:
                continue

            dist = np.linalg.norm(tooth_centers[i] - tooth_centers[tooth_idx])
            if dist < closest_tooth_dist:
                closest_tooth_dist = dist
                closest_tooth_idx = i

        if closest_tooth_idx is None:
            break

        labelled_teeth.append(closest_tooth_idx)
        tooth_idx = closest_tooth_idx
        counter += 1

    labelled_teeth.append(right_i)

    logger.info(f"Labelled teeth: {labelled_teeth}")

    assert len(labelled_teeth) == len(teeth_labels), \
        "Number of tooth labels does not match the number of teeth STL files"

    # saving the labelled teeth
    os.makedirs(output_folder_path, exist_ok=True)

    for i in range(len(teeth_labels)):
        tooth_label = teeth_labels[i]
        tooth_idx = labelled_teeth[i]

        tooth = tooth_objects[tooth_idx]

        filepath = os.path.join(output_folder_path, f'Tooth_{tooth_label}.stl')
        o3d.io.write_triangle_mesh(filepath, tooth)

        logger.info(f"Saved {filepath}")
