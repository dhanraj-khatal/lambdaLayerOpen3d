from dataclasses import dataclass
import json
from typing import List

@dataclass
class ToothMovementClass:
    intrusion: float
    extrusion: float
    mesial_tip: float
    distal_tip: float
    buccal_torque: float
    lingual_torque: float
    clockwise_rotation: float
    anticlockwise_rotation: float
    buccal_translation: float
    lingual_translation: float
    mesial_translation: float
    distal_translation: float
    tooth_num: float
    tooth_type: str

