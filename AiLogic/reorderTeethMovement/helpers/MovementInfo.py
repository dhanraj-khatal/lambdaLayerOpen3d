import numpy as np

from .ToothStepsTimeline import ToothStepsTimeline


class MovementInfo:
    def __init__(
            self,
            tooth_idx: int,
            is_maxillary: bool
    ):
        self.done = False
        self.tooth_idx = tooth_idx
        self.is_maxillary = is_maxillary
        self.initial_center = []
        self.current_center = []
        self.final_center = []

        self.translation_top = {
            'done': False,
            'distance_left': 0,
            'distance_translated': 0,
            'slope': [],
            'step_movement': 0,
            'max_step_movement': 0,
        }

        self.translation_side = {
            'done': False,
            'distance_left': 0,
            'distance_translated': 0,
            'slope': [],
            'step_movement': 0,
            'max_step_movement': 0,
        }

        self.translation_front = {
            'done': False,
            'distance_left': 0,
            'distance_translated': 0,
            'slope': [],
            'step_movement': 0,
            'max_step_movement': 0,
        }

        self.rotation = {
            'done': False,
            'angle_left': 0,
            'angle_rotated': 0,
            'axis': [],
            'step_movement': 0,
        }

        self.torque = {
            'done': False,
            'angle_left': 0,
            'angle_rotated': 0,
            'axis': [],
            'step_movement': 0,
        }

        self.tip = {
            'done': False,
            'angle_left': 0,
            'angle_rotated': 0,
            'axis': [],
            'step_movement': 0,
        }

    def __str__(self):
        return f'{self.tooth_idx=}\n' \
               f'{self.current_center=}\n' \
               f'{self.final_center=}\n' \
               f'{self.translation_top["distance_left"]=}\n' \
               f'{self.translation_front["distance_left"]=}\n' \
               f'{self.translation_side["distance_left"]=}\n' \
               f'{np.rad2deg(self.torque["angle_left"])=}\n' \
               f'{np.rad2deg(self.tip["angle_left"])=}\n' \
               f'{np.rad2deg(self.rotation["angle_left"])=}\n'

    def add_step_info(self, tooth_steps_timeline: ToothStepsTimeline):
        tooth_steps_timeline.total_steps += 1
        if self.translation_top['step_movement'] >= 0:
            tooth_steps_timeline.extrusion.append(self.translation_top['step_movement'])
            tooth_steps_timeline.intrusion.append(0)
        else:
            tooth_steps_timeline.intrusion.append(-self.translation_top['step_movement'])
            tooth_steps_timeline.extrusion.append(0)

        if self.translation_front['step_movement'] >= 0:
            tooth_steps_timeline.buccal_translation.append(self.translation_front['step_movement'])
            tooth_steps_timeline.lingual_translation.append(0)
        else:
            tooth_steps_timeline.lingual_translation.append(-self.translation_front['step_movement'])
            tooth_steps_timeline.buccal_translation.append(0)

        if self.translation_side['step_movement'] >= 0:
            if self.tooth_idx < 8 or 16 <= self.tooth_idx < 24:
                tooth_steps_timeline.mesial_translation.append(self.translation_side['step_movement'])
                tooth_steps_timeline.distal_translation.append(0)
            else:
                tooth_steps_timeline.distal_translation.append(self.translation_side['step_movement'])
                tooth_steps_timeline.mesial_translation.append(0)
        else:
            if self.tooth_idx < 8 or 16 <= self.tooth_idx < 24:
                tooth_steps_timeline.distal_translation.append(-self.translation_side['step_movement'])
                tooth_steps_timeline.mesial_translation.append(0)
            else:
                tooth_steps_timeline.mesial_translation.append(-self.translation_side['step_movement'])
                tooth_steps_timeline.distal_translation.append(0)

        if self.torque['step_movement'] >= 0:
            tooth_steps_timeline.lingual_torque.append(self.torque['step_movement'])
            tooth_steps_timeline.buccal_torque.append(0)
        else:
            tooth_steps_timeline.buccal_torque.append(-self.torque['step_movement'])
            tooth_steps_timeline.lingual_torque.append(0)

        if self.tip['step_movement'] >= 0:
            if self.tooth_idx < 8 or 16 <= self.tooth_idx < 24:
                tooth_steps_timeline.mesial_tip.append(self.tip['step_movement'])
                tooth_steps_timeline.distal_tip.append(0)
            else:
                tooth_steps_timeline.distal_tip.append(self.tip['step_movement'])
                tooth_steps_timeline.mesial_tip.append(0)
        else:
            if self.tooth_idx < 8 or 16 <= self.tooth_idx < 24:
                tooth_steps_timeline.distal_tip.append(-self.tip['step_movement'])
                tooth_steps_timeline.mesial_tip.append(0)
            else:
                tooth_steps_timeline.mesial_tip.append(-self.tip['step_movement'])
                tooth_steps_timeline.distal_tip.append(0)

        if self.rotation['step_movement'] >= 0:
            tooth_steps_timeline.anticlockwise_rotation.append(self.rotation['step_movement'])
            tooth_steps_timeline.clockwise_rotation.append(0)
        else:
            tooth_steps_timeline.clockwise_rotation.append(-self.rotation['step_movement'])
            tooth_steps_timeline.anticlockwise_rotation.append(0)

        self.translation_front['step_movement'] = 0
        self.translation_side['step_movement'] = 0
        self.translation_top['step_movement'] = 0
        self.torque['step_movement'] = 0
        self.tip['step_movement'] = 0
        self.rotation['step_movement'] = 0
