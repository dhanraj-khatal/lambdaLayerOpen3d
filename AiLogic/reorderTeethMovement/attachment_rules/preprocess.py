from .rules import *


def tooth_label(tooth_number):
    """
    assigns a label to a tooth according to what the tooth number is
    """

    teeth_dict = {"Maxillary Molar": [0, 1, 2, 13, 14, 15], "Maxillary Premolar": [3, 4, 11, 12],
                  "Maxillar Canine": [5, 10], "Maxillary Incisor": [6, 7, 8, 9],
                  "Mandibular Molar": [16, 17, 18, 29, 30, 31],
                  "Mandibular Premolar": [19, 20, 27, 28], "Mandibular Canine": [21, 26],
                  "Mandibular Incisor": [22, 23, 24, 25]}

    for key, value in teeth_dict.items():
        if tooth_number in value:
            return key


def tooth_number_from_file_name(file_path):
    """
    helps extract the tooth number from file name as used in json to data class function below
    this function extracts the file name
    """
    file_path_components = file_path.split(os.path.sep)  # changed so that package runs on windows as well as mac
    file_name_and_extension = file_path_components[-1].rsplit('.', 1)
    return file_name_and_extension[0]


def json_to_data_class(folder_path, file_name):
    """
    @param folder_path: path to the folder containing the json files
    @param file_name: name of the json file

    initialises a tooth with their attributes as extracted from the json
    """

    json_file_path = os.path.join(folder_path, file_name)
    with open(json_file_path, 'r') as json_file:
        json_data = json.load(json_file)
        file_name_components = file_name.split('_')
        tooth_number = file_name_components[1]

    return ToothMovementClass(
        sum(json_data["intrusion"]),
        sum(json_data["extrusion"]),
        sum(json_data["mesial_tip"]),
        sum(json_data["distal_tip"]),
        sum(json_data["buccal_torque"]),
        sum(json_data["lingual_torque"]),
        sum(json_data["clockwise_rotation"]),
        sum(json_data["anticlockwise_rotation"]),
        sum(json_data["buccal_translation"]),
        sum(json_data["lingual_translation"]),
        sum(json_data["mesial_translation"]),
        sum(json_data["distal_translation"]),
        file_name_components[1],
        tooth_label(int(tooth_number))
    )


def process_teeth_in_folder(folder_path):
    """
    initialises all teeth of the input folder with attributes as extracted from the json iterating over
    different teeth working with json_to_data_class
    """

    data_class_instances = []

    # Get a list of all JSON files in the folder
    json_files = [f for f in os.listdir(folder_path) if f.endswith('.json')]

    for json_file in json_files:
        data_class_instance = json_to_data_class(folder_path, json_file)
        data_class_instances.append(data_class_instance)

    return data_class_instances
