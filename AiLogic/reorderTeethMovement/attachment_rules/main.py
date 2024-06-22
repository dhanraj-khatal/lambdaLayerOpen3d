import preprocess
import os
import json
from .tooth_class import ToothMovementClass  # Import data class
from typing import List
import rules


if __name__ == "__main__":
    folder_path = r"C:\Users\eshri\Downloads\6949_teeth_steps_timeline\teeth_steps_timeline"  # Replace with the folder path
    tooth_data = preprocess.process_teeth_in_folder(folder_path)
    # call assign attachment here 
    # print(tooth_data)
    print(rules.assign_attachment(tooth_data))


    # print(attachment_list)
    # print(tooth_dict)
        