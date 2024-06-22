import csv
import os

import numpy as np

from .functions import save_as_json


class ToothStepsTimeline:
    def __init__(
            self,
            tooth_idx: int,
    ):
        self.done = False
        self.tooth_idx = tooth_idx
        self.total_steps = 0
        self.intrusion = []
        self.extrusion = []
        self.lingual_translation = []
        self.buccal_translation = []
        self.mesial_translation = []
        self.distal_translation = []

        self.buccal_torque = []
        self.lingual_torque = []
        self.mesial_tip = []
        self.distal_tip = []
        self.clockwise_rotation = []
        self.anticlockwise_rotation = []

    def __str__(self):
        return f'{self.tooth_idx=}\n' \
               f'{self.total_steps=}\n' \
               f'{self.intrusion=}\n' \
               f'{self.extrusion=}\n' \
               f'{self.lingual_translation=}\n' \
               f'{self.buccal_translation=}\n' \
               f'{self.mesial_translation=}\n' \
               f'{self.distal_translation=}\n' \
               f'{self.distal_tip=}\n' \
               f'{self.mesial_tip=}\n' \
               f'{self.lingual_torque=}\n' \
               f'{self.buccal_torque=}\n' \
               f'{self.clockwise_rotation=}\n' \
               f'{self.anticlockwise_rotation=}\n'

    def to_csv(
            self,
            filepath: os.path
    ):
        with open(filepath, 'w', newline='') as file:
            writer = csv.writer(file)
            headers = ["movement_type"] + [f'step_{str(step_count+1).zfill(3)}'
                                           for step_count in range(self.total_steps)]
            writer.writerow(headers)
            writer.writerow(['intrusion'] + [value for value in self.intrusion])
            writer.writerow(['extrusion'] + [value for value in self.extrusion])
            writer.writerow(['mesial_tip'] + [value for value in self.mesial_tip])
            writer.writerow(['distal_tip'] + [value for value in self.distal_tip])
            writer.writerow(['buccal_torque'] + [value for value in self.buccal_torque])
            writer.writerow(['lingual_torque'] + [value for value in self.lingual_torque])
            writer.writerow(['clockwise_rotation'] + [value for value in self.clockwise_rotation])
            writer.writerow(['anticlockwise_rotation'] + [value for value in self.anticlockwise_rotation])
            writer.writerow(['buccal_translation'] + [value for value in self.buccal_translation])
            writer.writerow(['lingual_translation'] + [value for value in self.lingual_translation])
            writer.writerow(['mesial_translation'] + [value for value in self.mesial_translation])
            writer.writerow(['distal_translation'] + [value for value in self.distal_translation])

    def to_json(
            self,
            filepath: os.path,
            total_step_count: int,
    ):
        difference = total_step_count - self.total_steps
        zeros_list = [0] * difference

        data = {
            'intrusion': self.intrusion + zeros_list,
            'extrusion': self.extrusion + zeros_list,
            'mesial_tip': self.mesial_tip + zeros_list,
            'distal_tip': self.distal_tip + zeros_list,
            'buccal_torque': self.buccal_torque + zeros_list,
            'lingual_torque': self.lingual_torque + zeros_list,
            'clockwise_rotation': self.clockwise_rotation + zeros_list,
            'anticlockwise_rotation': self.anticlockwise_rotation + zeros_list,
            'buccal_translation': self.buccal_translation + zeros_list,
            'lingual_translation': self.lingual_translation + zeros_list,
            'mesial_translation': self.mesial_translation + zeros_list,
            'distal_translation': self.distal_translation + zeros_list,
        }

        save_as_json(
            data=data,
            file_name=filepath
        )

    def from_dict(self, values: dict):
        self.extrusion = values['extrusion']
        self.intrusion = values['intrusion']
        self.buccal_translation = values['buccal_translation']
        self.lingual_translation = values['lingual_translation']
        self.mesial_translation = values['mesial_translation']
        self.distal_translation = values['distal_translation']
        self.buccal_torque = values['buccal_torque']
        self.lingual_torque = values['lingual_torque']
        self.clockwise_rotation = values['clockwise_rotation']
        self.anticlockwise_rotation = values['anticlockwise_rotation']
        self.mesial_tip = values['mesial_tip']
        self.distal_tip = values['distal_tip']

        self.total_steps = len(self.extrusion)

