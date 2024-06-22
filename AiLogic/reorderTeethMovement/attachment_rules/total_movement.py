import json
from copy import deepcopy
import glob
import sys


def calculate_total_movement(input_folder, output_json_file):
    """
    Input :
        input_folder - folder containing all json files having details regarding tooth movement for each step.
        output_json_file - json file name to save the final result.

    Output : A list of parameters for each tooth_id with parameters describing the
            total movement done on that particular tooth.

    The function saves the final tooth movement as output_json_file and returns the list for the same as well.
    """

    final_params = {
        "tooth_id": 0,
        "total_rotation1": 0,
        "total_rotation2": 0,
        "total_x_dist": 0,
        "total_y_dist": 0,
        "total_z_dist": 0,
        "total_tooth_distance": 0,
        "total_tip_rotation": 0,
        "total_torque_rotation": 0,
    }

    # dictionary of mapping of current parameters to the final parameters
    params = {
        "rotation1": "total_rotation1",
        "rotation2": "total_rotation2",
        "x_dist": "total_x_dist",
        "y_dist": "total_y_dist",
        "z_dist": "total_z_dist",
        "tooth_distance": "total_tooth_distance",
        "tip_rotation": "total_tip_rotation",
        "torque_rotation": "total_torque_rotation",
    }

    # initializing tooth id for final output
    total_movement = [deepcopy(final_params) for i in range(1, 33)]
    for i in range(32):
        total_movement[i]["tooth_id"] = i + 1

    # reading all json files and calculating final parameters.
    files_list = [f for f in glob.glob(input_folder + "/*.json")]
    for file in files_list:
        tooth_movement_step = json.load(open(file))

        for i in range(len(tooth_movement_step)):
            tooth_id = tooth_movement_step[i]["tooth_id"] - 1
            for param in params:
                total_movement[tooth_id][params[param]] += tooth_movement_step[i][param]

    # for saving json file
    with open(output_json_file + ".json", "w") as output:
        json.dump(total_movement, output)

    return total_movement


if __name__ == "__main__":
    input_folder = sys.argv[1]
    output_json_file = sys.argv[2]
    print(input_folder, output_json_file)

    calculate_total_movement(input_folder, output_json_file)