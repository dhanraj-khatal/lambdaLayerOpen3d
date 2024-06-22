from .tooth_class import ToothMovementClass
import os
import json
from typing import List, Dict


TOOTH_TYPE_TO_NUMBER = {
    "Maxillary Molar": [0, 1, 2, 13, 14, 15],
    "Maxillary First 2 Molars": [1, 2, 13, 14],
    "Maxillary Premolar": [3, 4, 11, 12],
    "Maxillar Canine": [5, 10],
    "Maxillary Incisor": [6, 7, 8, 9],
    "Mandibular Molar": [16, 17, 18, 29, 30, 31],
    "Mandibular First 2 Molars": [17, 18, 29, 30],
    "Mandibular Premolar": [19, 20, 27, 28],
    "Mandibular Canine": [21, 26],
    "Mandibular Incisor": [22, 23, 24, 25],
}


def assign_attachment(tooth_data: List[ToothMovementClass]) -> Dict[int, List[str]]:
    """
    calls all the different functions to implement their rules and populate the
    tooth attachments dictionary with the teeth that needs attachments
    """

    tooth_attachments = {i: [] for i in range(1, 33)}

    tooth_attachments = sem_circ_check_pre_ant(tooth_data, tooth_attachments)
    tooth_attachments = sem_circ_check_pre_post(tooth_data, tooth_attachments)
    tooth_attachments = hor_rect_check(tooth_data, tooth_attachments)
    tooth_attachments = semi_circ_check_mol(tooth_data, tooth_attachments)
    tooth_attachments = pow_ridge(tooth_data, tooth_attachments)
    tooth_attachments = circular_sub_mov(tooth_data, tooth_attachments)
    tooth_attachments = vertical_rectangular_attn(tooth_data, tooth_attachments)
    tooth_attachments = ext_att_only_extrude(tooth_data, tooth_attachments)
    tooth_attachments = ext_att_extrude_and_translate(tooth_data, tooth_attachments)
    tooth_attachments = bite_ramp(tooth_data, tooth_attachments)

    return tooth_attachments


def sem_circ_check_pre_ant(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
        1. Anterior( Maxillary Incisors: [6, 7, 8, 9],Mandibular Incisors: [22, 23, 24, 25],Maxillary Canines: [5, 10],Mandibular Canines: [21, 26 ]  )
     intrude (Intrusion) between 0.5mm to 2.5mm and rotation (Clockwise_rotation / Anticlockwise_rotation)  of premolars is less than 5 degrees.

    Apply Attachments to:
    Premolars

    """

    # do maxillary first
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in [
            "Maxillary Incisor",
            "Maxillary Canine",
        ]:
            # check incisors and canines condition first
            if 0.5 <= tooth.intrusion <= 2.5:
                for i, tooth_in_loop in enumerate(tooth_data):
                    if tooth_in_loop.tooth_type in ["Maxillary Premolar"]:
                        # dont add it again
                        if (
                            "semiCircularMolarAndPremolar"
                            not in tooth_attachments[int(tooth_in_loop.tooth_num)]
                        ):
                            # check premolar condition
                            if (tooth_in_loop.anticlockwise_rotation <= 5) and (
                                tooth_in_loop.clockwise_rotation <= 5
                            ):
                                tooth_attachments[int(tooth_in_loop.tooth_num)].append(
                                    "semiCircularMolarAndPremolar"
                                )

    # do Mandibular first
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in [
            "Mandibular Incisor",
            "Mandibular Canine",
        ]:
            # check incisors and canines condition first
            if 0.5 <= tooth.intrusion <= 2.5:
                for i, tooth_in_loop in enumerate(tooth_data):
                    if tooth_in_loop.tooth_type in ["Mandibular Premolar"]:
                        # dont add it again
                        if (
                            "semiCircularMolarAndPremolar"
                            not in tooth_attachments[int(tooth_in_loop.tooth_num)]
                        ):
                            # check premolar condition
                            if (tooth_in_loop.anticlockwise_rotation <= 5) and (
                                tooth_in_loop.clockwise_rotation <= 5
                            ):
                                tooth_attachments[int(tooth_in_loop.tooth_num)].append(
                                    "semiCircularMolarAndPremolar"
                                )

    return tooth_attachments


def sem_circ_check_pre_post(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
         Posterior ( Maxillary Premolars: [3, 4, 11, 12], ,Mandibular Premolars: [19, 20, 27, 28] )
    Expand ( Buccal_translation)  more than 1mm
        Apply Attachments to:  Premolars
    """

    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in ["Maxillary Premolar", "Mandibular Premolar"]:
            if tooth.buccal_translation > 1:
                # dont add it again
                if (
                    "semiCircularMolarAndPremolar"
                    not in tooth_attachments[int(tooth.tooth_num)]
                ):
                    tooth_attachments[int(tooth.tooth_num)].append(
                        "semiCircularMolarAndPremolar"
                    )

    return tooth_attachments


def hor_rect_check(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    Anterior( Maxillary Incisors Mandibular Incisors: [22, 23, 24, 25],Maxillary Canines: [5, 10],Mandibular Canines: [21, 26 ]  )
    intrude (Intrusion)  between 2.5mm to 4mm.

    Apply Attachments to: First and second molars

    """

    # check maxillary first
    for _, tooth in enumerate(tooth_data):
        if tooth.tooth_type in [
            "Maxillary Incisor",
            "Maxillary Canine",
        ]:
            if 2.5 <= tooth.intrusion <= 4:
                for molar in TOOTH_TYPE_TO_NUMBER["Maxillary First 2 Molars"]:
                    # dont add it again
                    if "deepBiteAttachmentHorizontal" not in tooth_attachments[molar]:
                        tooth_attachments[molar].append("deepBiteAttachmentHorizontal")

    # check mandibular
    for _, tooth in enumerate(tooth_data):
        if tooth.tooth_type in [
            "Mandibular Incisor",
            "Mandibular Canine",
        ]:
            if 2.5 <= tooth.intrusion <= 4:
                for molar in TOOTH_TYPE_TO_NUMBER["Mandibular First 2 Molars"]:
                    # dont add it again
                    if "deepBiteAttachmentHorizontal" not in tooth_attachments[molar]:
                        tooth_attachments[molar].append("deepBiteAttachmentHorizontal")

    return tooth_attachments


def semi_circ_check_mol(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    1. Posterior( Maxillary Molars, Mandibular Molars)  expand ( Buccal_translation)  more than 1mm.

    Apply Attachments to: Molars

    """

    # check maxillary first
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in ["Maxillary Molar"]:
            if tooth.buccal_translation > 1:
                # dont add it again
                if (
                    "semiCircularMolarAndPremolar"
                    not in tooth_attachments[int(tooth.tooth_num)]
                ):
                    tooth_attachments[int(tooth.tooth_num)].append(
                        "semiCircularMolarAndPremolar"
                    )

    # check mandibular next
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in ["Mandibular Molar"]:
            if tooth.buccal_translation > 1:
                # dont add it again
                if (
                    "semiCircularMolarAndPremolar"
                    not in tooth_attachments[int(tooth.tooth_num)]
                ):
                    tooth_attachments[int(tooth.tooth_num)].append(
                        "semiCircularMolarAndPremolar"
                    )

    return tooth_attachments


def pow_ridge(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    1. Triggered on Maxillary/ mandibular  Incisors More than 3 degree of torque ( Buccal_torque )

    Apply Attachments to: Incisors only not canines

    """

    # check maxillary first
    for _, tooth in enumerate(tooth_data):
        if tooth.tooth_type == "Maxillary Incisor":
            if tooth.buccal_torque > 3:
                tooth_attachments[int(tooth.tooth_num)].append("halfCylinder")

    # check mandibular next
    for _, tooth in enumerate(tooth_data):
        if tooth.tooth_type == "Mandibular Incisor":
            if tooth.buccal_torque > 3:
                tooth_attachments[int(tooth.tooth_num)].append("halfCylinder")

    return tooth_attachments


def circular_sub_mov(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    Intrusion of  Mandibular Incisors: [22, 23, 24, 25] more than 0.5mm.
    (Triggered on Lower Incisor Lingual Surface - circular subtractive attachment)

    Apply Attachments to: lower Incisors

    """
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type == "Mandibular Incisor":
            if tooth.intrusion > 0.5:
                tooth_attachments[int(tooth.tooth_num)].append("lowerIncisorCircle")

    return tooth_attachments


def vertical_rectangular_attn(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    Rotation (Clockwise_rotation / Anticlockwise_rotation)  greater than 5 degrees.

    Apply Attachments to: Respective tooth ( canines to Molars ) which ever tooth has more than 5 deg rotation clockwise / anticlockwise

    """

    # check maxillary first
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in [
            "Maxillary Canine",
            "Maxillary Premolar",
            "Maxillary Molar",
        ]:
            if tooth.clockwise_rotation > 5 or tooth.anticlockwise_rotation > 5:
                if (
                    "deepBiteAttachmentVertical"
                    not in tooth_attachments[int(tooth.tooth_num)]
                ):
                    tooth_attachments[int(tooth.tooth_num)].append(
                        "deepBiteAttachmentVertical"
                    )

    # check Mandibular first
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in [
            "Mandibular Canine",
            "Mandibular Premolar",
            "Mandibular Molar",
        ]:
            if tooth.clockwise_rotation > 5 or tooth.anticlockwise_rotation > 5:
                if (
                    "deepBiteAttachmentVertical"
                    not in tooth_attachments[int(tooth.tooth_num)]
                ):
                    tooth_attachments[int(tooth.tooth_num)].append(
                        "deepBiteAttachmentVertical"
                    )

    return tooth_attachments


def ext_att_only_extrude(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    1. Incisors ( Maxillary Incisors: [6, 7, 8, 9],Mandibular Incisors: [22, 23, 24, 25])   greater than 0.5mm. Of Extrusion
    Apply Attachments to: Maxillary, Mandibular incisors
    """

    # check maxillary first
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type in ["Maxillary Incisor", "Mandibular Incisor"]:
            if tooth.extrusion > 0.5:
                if "openBiteExtrusion" not in tooth_attachments[int(tooth.tooth_num)]:
                    tooth_attachments[int(tooth.tooth_num)].append("openBiteExtrusion")

    return tooth_attachments


def ext_att_extrude_and_translate(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    2. If upper incisors ( Maxillary Incisors: [6, 7, 8, 9]) are being pushed ahead ( buccal translation ) to create space for an instanding tooth.
    ( Buccal_translation ) more than 1 mm
    Apply Attachments to: upper Incisors
    """
    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type == "Maxillary Incisor":
            if tooth.buccal_translation > 0.5:
                if "openBiteExtrusion" not in tooth_attachments[int(tooth.tooth_num)]:
                    tooth_attachments[int(tooth.tooth_num)].append("openBiteExtrusion")

    return tooth_attachments


def bite_ramp(
    tooth_data: List[ToothMovementClass], tooth_attachments: Dict[int, List[str]]
) -> Dict[int, List[str]]:
    """
    1- Gets triggered on the  lingual of upper Incisors  when
    Movement is  Intrusion of lower Incisors (Mandibular Incisors: [22, 23, 24, 25] )  more than 1.5 mm

    Apply Attachments to: Upper incisors on lingual side = "Maxillary Incisors: [6, 7, 8, 9]"
    """

    for i, tooth in enumerate(tooth_data):
        if tooth.tooth_type == "Mandibular Incisor":
            if tooth.intrusion > 1.5:
                for max_incisor in TOOTH_TYPE_TO_NUMBER["Maxillary Incisor"]:
                    if "precisionBiteRamp" not in tooth_attachments[max_incisor]:
                        tooth_attachments[max_incisor].append(
                            "precisionBiteRamp"
                        )

    return tooth_attachments
