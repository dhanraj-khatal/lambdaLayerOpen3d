import errno
import time

import AiLogic.reorderTeethMovement.attachment_rules as attachment_rules
from AiLogic.reorderTeethMovement.helpers import *
from AiLogic.reorderTeethMovement.helpers.ToothAxis import ToothAxis
from AiLogic.reorderTeethMovement.helpers.ToothStepsTimeline import ToothStepsTimeline

MAX_STEPS_COUNT = 50
MILESTONE_COUNT = 1


def get_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--initial-folder",
        "-if",
        help="path to a particular case folder with initial teeth stls",
    )

    parser.add_argument(
        "--initial-points-folder",
        "-ipf",
        help="path to the initial points folder containing the points in JSON format",
    )

    parser.add_argument(
        "--initial-triangles-folder",
        "-itf",
        help="path to the initial triangles folder containing the triangle stls for each tooth",
    )

    parser.add_argument(
        "--initial-circles-folder",
        "-icf",
        help="path to the output folder of gum snapping points api containing the files as Tooth_i_circle.json",
    )

    parser.add_argument(
        "--initial-axes-folder",
        "-iaf",
        help="path to the initial axes folder containing the axes json for each tooth",
    )

    parser.add_argument(
        "--final-axes-folder",
        "-faf",
        help="path to the final axes folder containing the axes json for each tooth",
    )

    parser.add_argument(
        "--steps-timeline-folder",
        "-stf",
        help="path to the timeline folder containing the timeline json for each tooth, "
             "named as `Tooth_{i}_steps_timeline.json`",
    )

    parser.add_argument(
        "--save-updated-steps-timeline",
        "-sust",
        help="pass either 1 or 0 to either save the updated steps timeline to the output folder or not",
        default=1,
    )

    parser.add_argument(
        "--model-folder",
        "-mf",
        help="path to the model folder containing tooth_config.ini",
    )

    parser.add_argument(
        "--output-folder",
        "-of",
        help="path to save step GLTFs and other log/json files",
    )

    arguments = parser.parse_args()

    return arguments


def get_tooth_axes_through_tool(filepath: str):
    axes_picked = get_json_object(filepath)

    crown_center = swap_z_and_y_coordinates(axes_picked["tooth_crown_center"])
    top_point = swap_z_and_y_coordinates(axes_picked["top_point"])
    front_point = swap_z_and_y_coordinates(axes_picked["front_point"])
    side_point = swap_z_and_y_coordinates(axes_picked["side_point"])

    top_axis: ToothAxis = ToothAxis(first_point=crown_center, second_point=top_point)
    front_axis: ToothAxis = ToothAxis(first_point=crown_center, second_point=front_point)
    side_axis: ToothAxis = ToothAxis(first_point=crown_center, second_point=side_point)

    return top_axis, front_axis, side_axis


def reorderTeethMovementOperation(
        save_updated_steps_timeline: bool,
        initial_folder: str,
        initial_points_folder: str,
        initial_triangles_folder: str,
        initial_circles_folder: str,
        initial_axes_folder: str,
        final_axes_folder: str,
        steps_timeline_folder: str,
        model_folder: str,
        output_folder: str,
):
    start_time = time.time()
    logger = logging.getLogger()

    if not os.path.isdir(initial_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_folder)

    if not os.path.isdir(initial_points_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_points_folder)

    if not os.path.isdir(initial_triangles_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_triangles_folder)

    if not os.path.isdir(initial_circles_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_circles_folder)

    if not os.path.isdir(initial_axes_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_axes_folder)

    if not os.path.isdir(final_axes_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), final_axes_folder)

    if not os.path.isdir(steps_timeline_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), steps_timeline_folder)

    if not os.path.isdir(model_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), model_folder)

    renew_folder(output_folder)

    Constants.CREATE_TEETH_STEPS_TIMELINES = save_updated_steps_timeline

    st = time.time()
    initial_teeth = Teeth(
        case_folder=initial_folder,
        points_folder=initial_points_folder,
        triangles_folder=initial_triangles_folder,
        circles_folder=initial_circles_folder,
        axes_folder=initial_axes_folder,
        model_folder=model_folder,
        config_file=os.path.join(model_folder, "tooth_config.ini"),
    )
    logger.info(f'Initial teeth imported {time.time() - st}')

    # Save the deepcopy of the final and initial axes for each tooth
    final_teeth_axes = []
    initial_teeth_axes = []
    for tooth_idx in range(Constants.MAX_TEETH_COUNT):
        axes_path = os.path.join(final_axes_folder, f"Tooth_{tooth_idx + 1}_axes_points.json")
        if not os.path.isfile(axes_path):
            final_teeth_axes.append(None)
            initial_teeth_axes.append(None)
            continue

        top_axis, front_axis, side_axis = get_tooth_axes_through_tool(axes_path)
        final_teeth_axes.append({
            'top_axis': copy.deepcopy(top_axis.get_slope()),
            'front_axis': copy.deepcopy(front_axis.get_slope()),
            'side_axis': copy.deepcopy(side_axis.get_slope()),
        })

        initial_teeth_axes.append({
            'top_axis': copy.deepcopy(initial_teeth.teeth_objects[tooth_idx].top_axis.get_slope()),
            'front_axis': copy.deepcopy(initial_teeth.teeth_objects[tooth_idx].front_axis.get_slope()),
            'side_axis': copy.deepcopy(initial_teeth.teeth_objects[tooth_idx].side_axis.get_slope())
        })

    # creating different output folders
    save_path = output_folder

    logs_path = os.path.join(save_path, "logs/")
    front_point_logs_path = os.path.join(save_path, "front_point_logs/")
    teeth_steps_path = os.path.join(save_path, "teeth_steps/")
    teeth_steps_timelines_path = os.path.join(save_path, "teeth_steps_timeline/")
    teeth_triangle_steps_path = os.path.join(save_path, "teeth_triangle_steps/")
    teeth_circle_steps_path = os.path.join(save_path, "teeth_circle_steps/")

    renew_folder(logs_path)
    renew_folder(front_point_logs_path)
    renew_folder(teeth_triangle_steps_path)
    renew_folder(teeth_circle_steps_path)
    renew_folder(teeth_steps_path)

    if Constants.CREATE_TEETH_STEPS_TIMELINES:
        renew_folder(teeth_steps_timelines_path)

    teeth_steps_timelines = []
    for tooth_idx in range(Constants.MAX_TEETH_COUNT):
        tooth_id = tooth_idx + 1
        tooth: Tooth = initial_teeth.teeth_objects[tooth_idx]
        if tooth.is_dummy:
            teeth_steps_timelines.append(None)
            continue

        timeline_filepath = os.path.join(steps_timeline_folder, f'Tooth_{tooth_id}_steps_timeline.json')
        if not os.path.isfile(timeline_filepath):
            teeth_steps_timelines.append(None)
            assert False, f'File not found: {timeline_filepath}'

        # read the timeline json file
        tooth_steps_timeline: ToothStepsTimeline = ToothStepsTimeline(tooth_idx)
        timeline_values: dict = get_json_object(filepath=timeline_filepath)

        tooth_steps_timeline.from_dict(timeline_values)
        teeth_steps_timelines.append(tooth_steps_timeline)

    tooth_movement_done = [False for _ in range(Constants.MAX_TEETH_COUNT)]

    step_count = 0
    with open(os.path.join(logs_path, f'log{step_count}.json'), "w") as log_file:
        log_file.write('[]')

    initial_teeth.save_front_point_logs(
        step_count=step_count,
        front_point_logs_path=front_point_logs_path,
    )

    if Constants.CREATE_TEETH_STEPS:
        # save teeth GLTFs per step
        initial_teeth.export_teeth_objects(
            step_count=step_count,
            teeth_steps_path=teeth_steps_path,
            teeth_circle_steps_path=teeth_circle_steps_path,
            teeth_triangle_steps_path=teeth_triangle_steps_path,
        )

    for step_idx in range(1, MAX_STEPS_COUNT):
        action_logs = []

        for tooth_idx in range(Constants.MAX_TEETH_COUNT):
            tooth: Tooth = initial_teeth.teeth_objects[tooth_idx]
            if tooth.is_dummy:
                tooth_movement_done[tooth_idx] = True
                continue

            tooth_steps_timeline: ToothStepsTimeline = teeth_steps_timelines[tooth_idx]
            if step_idx >= tooth_steps_timeline.total_steps:
                tooth_movement_done[tooth_idx] = True
                continue

            # applying torque
            torque = 0
            if step_idx < len(tooth_steps_timeline.buccal_torque) and \
                    abs(tooth_steps_timeline.buccal_torque[step_idx]) > 0:
                torque = -tooth_steps_timeline.buccal_torque[step_idx]
                tooth.rotate_using_quaternion(
                    axis=final_teeth_axes[tooth_idx]['side_axis'],
                    angle=np.deg2rad(torque),
                    rotating_center=tooth.crown['center_point'],
                    move_back=True,
                )

            elif step_idx < len(tooth_steps_timeline.lingual_torque) and \
                    abs(tooth_steps_timeline.lingual_torque[step_idx]) > 0:
                torque = tooth_steps_timeline.lingual_torque[step_idx]
                tooth.rotate_using_quaternion(
                    axis=final_teeth_axes[tooth_idx]['side_axis'],
                    angle=np.deg2rad(torque),
                    rotating_center=tooth.crown['center_point'],
                    move_back=True,
                )

            # applying tip
            tip = 0
            if step_idx < len(tooth_steps_timeline.mesial_tip) and \
                    abs(tooth_steps_timeline.mesial_tip[step_idx]) > 0:
                if tooth_idx < 8 or 16 <= tooth_idx < 24:
                    tip = tooth_steps_timeline.mesial_tip[step_idx]
                else:
                    tip = -tooth_steps_timeline.mesial_tip[step_idx]

                tooth.rotate_using_quaternion(
                    axis=final_teeth_axes[tooth_idx]['front_axis'],
                    angle=np.deg2rad(tip),
                    rotating_center=tooth.crown['center_point'],
                    move_back=True,
                )

            elif step_idx < len(tooth_steps_timeline.distal_tip) and \
                    abs(tooth_steps_timeline.distal_tip[step_idx]) > 0:
                if tooth_idx < 8 or 16 <= tooth_idx < 24:
                    tip = -tooth_steps_timeline.distal_tip[step_idx]
                else:
                    tip = tooth_steps_timeline.distal_tip[step_idx]

                tooth.rotate_using_quaternion(
                    axis=final_teeth_axes[tooth_idx]['front_axis'],
                    angle=np.deg2rad(tip),
                    rotating_center=tooth.crown['center_point'],
                    move_back=True,
                )

            # applying rotation
            rotation = 0
            if step_idx < len(tooth_steps_timeline.clockwise_rotation) and \
                    abs(tooth_steps_timeline.clockwise_rotation[step_idx]) > 0:
                rotation = -tooth_steps_timeline.clockwise_rotation[step_idx]
                tooth.rotate_using_quaternion(
                    axis=final_teeth_axes[tooth_idx]['top_axis'],
                    angle=np.deg2rad(rotation),
                    rotating_center=tooth.crown['center_point'],
                    move_back=True,
                )

            elif step_idx < len(tooth_steps_timeline.anticlockwise_rotation) and \
                    abs(tooth_steps_timeline.anticlockwise_rotation[step_idx]) > 0:
                rotation = tooth_steps_timeline.anticlockwise_rotation[step_idx]
                tooth.rotate_using_quaternion(
                    axis=final_teeth_axes[tooth_idx]['top_axis'],
                    angle=np.deg2rad(rotation),
                    rotating_center=tooth.crown['center_point'],
                    move_back=True,
                )

            # applying intrusion/extrusion
            translation_top = 0
            if step_idx < len(tooth_steps_timeline.extrusion) and \
                    abs(tooth_steps_timeline.extrusion[step_idx]) > 0:
                translation_top = tooth_steps_timeline.extrusion[step_idx]
                movement = translation_top * initial_teeth_axes[tooth_idx]['top_axis']
                tooth.translate_coordinates(
                    x_dist=movement[0],
                    y_dist=movement[1],
                    z_dist=movement[2],
                )

            elif step_idx < len(tooth_steps_timeline.intrusion) and \
                    abs(tooth_steps_timeline.intrusion[step_idx]) > 0:
                translation_top = -tooth_steps_timeline.intrusion[step_idx]
                movement = translation_top * initial_teeth_axes[tooth_idx]['top_axis']
                tooth.translate_coordinates(
                    x_dist=movement[0],
                    y_dist=movement[1],
                    z_dist=movement[2],
                )

            # applying mesial/distal translation
            translation_side = 0
            if step_idx < len(tooth_steps_timeline.mesial_translation) and \
                    abs(tooth_steps_timeline.mesial_translation[step_idx]) > 0:
                if tooth_idx < 8 or 16 <= tooth_idx < 24:
                    translation_side = tooth_steps_timeline.mesial_translation[step_idx]
                else:
                    translation_side = -tooth_steps_timeline.mesial_translation[step_idx]

                movement = translation_side * initial_teeth_axes[tooth_idx]['side_axis']
                tooth.translate_coordinates(
                    x_dist=movement[0],
                    y_dist=movement[1],
                    z_dist=movement[2],
                )

            elif step_idx < len(tooth_steps_timeline.distal_translation) and \
                    abs(tooth_steps_timeline.distal_translation[step_idx]) > 0:
                if tooth_idx < 8 or 16 <= tooth_idx < 24:
                    translation_side = -tooth_steps_timeline.distal_translation[step_idx]
                else:
                    translation_side = tooth_steps_timeline.distal_translation[step_idx]

                movement = translation_side * initial_teeth_axes[tooth_idx]['side_axis']
                tooth.translate_coordinates(
                    x_dist=movement[0],
                    y_dist=movement[1],
                    z_dist=movement[2],
                )

            # applying buccal/lingual translation
            translation_front = 0
            if step_idx < len(tooth_steps_timeline.buccal_translation) and \
                    abs(tooth_steps_timeline.buccal_translation[step_idx]) > 0:
                translation_front = tooth_steps_timeline.buccal_translation[step_idx]
                movement = translation_front * initial_teeth_axes[tooth_idx]['front_axis']
                tooth.translate_coordinates(
                    x_dist=movement[0],
                    y_dist=movement[1],
                    z_dist=movement[2],
                )

            elif step_idx < len(tooth_steps_timeline.lingual_translation) and \
                    abs(tooth_steps_timeline.lingual_translation[step_idx]) > 0:
                translation_front = -tooth_steps_timeline.lingual_translation[step_idx]
                movement = translation_front * initial_teeth_axes[tooth_idx]['front_axis']
                tooth.translate_coordinates(
                    x_dist=movement[0],
                    y_dist=movement[1],
                    z_dist=movement[2],
                )

            log = {
                'tooth_id': tooth_idx + 1,
                'current_center': list(tooth.crown['center_point']),
                'translation_top': translation_top,
                'translation_side': translation_side,
                'translation_front': translation_front,
                'rotation': rotation,
                'tip': tip,
                'torque': torque,
            }

            action_logs.append(log)

        if all(tooth_movement_done):
            break

        step_count += 1
        logger.info(f'Step count: {step_count}')

        json_obj = json.dumps(action_logs, indent=4)
        with open(os.path.join(logs_path, f'log{step_count}.json'), "w") as log_file:
            log_file.write(json_obj)

        initial_teeth.save_front_point_logs(
            step_count=step_count,
            front_point_logs_path=front_point_logs_path,
        )

        if Constants.CREATE_TEETH_STEPS:
            # save teeth GLTFs per step
            initial_teeth.export_teeth_objects(
                step_count=step_count,
                teeth_steps_path=teeth_steps_path,
                teeth_circle_steps_path=teeth_circle_steps_path,
                teeth_triangle_steps_path=teeth_triangle_steps_path,
            )

        print('Saved step:', step_idx)
        logger.info('Saved step:')

    # saving updated teeth steps timeline
    if Constants.CREATE_TEETH_STEPS_TIMELINES:
        for tooth_idx in range(Constants.MAX_TEETH_COUNT):
            tooth_id = 1 + tooth_idx
            tooth_steps_timeline = teeth_steps_timelines[tooth_idx]
            if tooth_steps_timeline is None:
                continue

            tooth_steps_timeline.to_json(
                filepath=os.path.join(teeth_steps_timelines_path, f'Tooth_{tooth_id}_steps_timeline.json'),
                total_step_count=step_count
            )
            logger.info(f'Saved Tooth_{tooth_id}_steps_timeline.json')

    logger.info(f'Checking attachment rules')
    attachment_preprocess_data = attachment_rules.preprocess.process_teeth_in_folder(teeth_steps_timelines_path)
    attachments = attachment_rules.rules.assign_attachment(attachment_preprocess_data)
    save_as_json(attachments, os.path.join(save_path, 'attachments.json'))

    end_time = time.time()
    logger.info(f'Compute time taken: {(end_time - start_time)}')
