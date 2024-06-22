import configparser
import copy

from joblib import Parallel, delayed

from . import Tooth
from .MovementInfo import MovementInfo
from .ToothEasyMesh import ToothEasyMesh
from .functions import *


class Teeth:
    def __init__(
            self,
            config_file,
            case_folder,
            points_folder,
            triangles_folder="",
            circles_folder="",
            model_folder="",
            axes_folder="",
            final_teeth=None,
            is_final=False,
    ):
        self.config_parser = configparser.ConfigParser()
        self.config_parser.read(config_file)

        self.maxillary_count = 16
        self.mandibular_count = 16
        self.teeth_objects = []
        self.case_folder = case_folder
        self.points_folder = points_folder
        self.triangles_folder = triangles_folder
        self.circles_folder = circles_folder
        self.axes_folder = axes_folder
        self.model_folder = model_folder
        self.is_final = is_final

        self.import_teeth_objects()

        if self.is_final:
            self.calculate_collision_distance()
        else:
            self.calculate_collision_distance()
            self.final_teeth = final_teeth
            # self.reorder_teeth_circle_points()

    def import_teeth_objects(self):
        for tooth_id in range(Constants.LOWER_IDX, Constants.UPPER_IDX):
            tooth_idx = tooth_id - 1
            tooth = Tooth(
                tooth_id=tooth_id,
                file_name=Constants.TEETH_FILE_NAME,
                case_folder=self.case_folder,
                points_folder=self.points_folder,
                triangles_folder=self.triangles_folder,
                circles_folder=self.circles_folder,
                axes_folder=self.axes_folder,
                model_folder=self.model_folder,
                config_parser=self.config_parser,
                is_final=self.is_final
            )

            if tooth.is_dummy:
                if tooth_idx in Constants.MAXILLARY_RANGE:
                    self.maxillary_count -= 1
                else:
                    self.mandibular_count -= 1

            tooth.tooth.compute_vertex_normals()

            self.teeth_objects.append(tooth)

        self.find_closest_neighbour_teeth(low=0, high=16, other_low=16, other_high=32)
        self.find_closest_neighbour_teeth(low=16, high=32, other_low=0, other_high=16)

        # if not self.is_final:
        #     for tooth_idx in range(Constants.MAX_TEETH_COUNT):
        #         tooth: Tooth = self.teeth_objects[tooth_idx]
        #         if tooth.is_dummy:
        #             continue
        #
        #         self.remove_high_elevation_points_of_segmentation_line(tooth_idx=tooth_idx)
        #         tooth.get_snapping_points_on_the_tooth_surface()

    # def remove_high_elevation_points_of_segmentation_line(
    #         self,
    #         tooth_idx,
    # ):
    #     tooth: Tooth = self.teeth_objects[tooth_idx]
    #     if tooth.is_dummy:
    #         return
    #
    #     all_circle_points = tooth.circle_points["list"]
    #     n_circle_points = len(all_circle_points)
    #     ncp = []
    #
    #     # remove the points which are touching the circle points of neighbouring teeth
    #     is_gum_points = [True] * n_circle_points
    #     for neighbour_tooth_idx in tooth.same_jaw_neighbours:
    #         if neighbour_tooth_idx == -1:
    #             continue
    #
    #         neighbour_tooth: Tooth = self.teeth_objects[neighbour_tooth_idx]
    #         if neighbour_tooth.is_dummy:
    #             continue
    #
    #         neighbour_circle_points = neighbour_tooth.circle_points["original_list"]
    #         ncp += list(neighbour_circle_points)
    #
    #         for curr_i in range(n_circle_points):
    #             circle_point = all_circle_points[curr_i]
    #
    #             # calculate the minimum distance of circle_point from the neighbour circle points using numpy
    #             min_distance = np.min(np.linalg.norm(neighbour_circle_points - circle_point, axis=1))
    #
    #             if min_distance < Constants.GUMLINE_NEIGHBOURING_DISTANCE_THRESHOLD:
    #                 is_gum_points[curr_i] = not is_gum_points[curr_i]
    #
    #     gum_points = []
    #     not_gum_points = []
    #     for i in range(n_circle_points):
    #         if is_gum_points[i]:
    #             gum_points.append(all_circle_points[i])
    #         else:
    #             not_gum_points.append(all_circle_points[i])
    #
    #     if len(not_gum_points) < 5:
    #         return
    #
    #     # find the open and close brackets
    #     open_brackets = []
    #     close_brackets = []
    #     for i in range(n_circle_points):
    #         prev_i = i
    #         curr_i = (i + 1) % n_circle_points
    #
    #         if is_gum_points[prev_i] and not is_gum_points[curr_i]:
    #             open_brackets.append(prev_i)
    #         elif not is_gum_points[prev_i] and is_gum_points[curr_i]:
    #             close_brackets.append(prev_i)
    #
    #     tooth_easy_mesh: ToothEasyMesh = tooth.tooth_easy_mesh
    #
    #     n_brackets = len(open_brackets)
    #     new_circle_points = []
    #
    #     if open_brackets[0] < close_brackets[0]:
    #         prev_close_bracket = 0
    #         for i in range(n_brackets):
    #             open_bracket = open_brackets[i]
    #             close_bracket = close_brackets[i]
    #
    #             for j in range(prev_close_bracket, open_bracket):
    #                 new_circle_points.append(all_circle_points[j])
    #
    #             source = tooth_easy_mesh.find_closest_point(all_circle_points[open_bracket])
    #             sink = tooth_easy_mesh.find_closest_point(all_circle_points[close_bracket])
    #             path = tooth_easy_mesh.find_geodesic_path(
    #                 source_point_id=source,
    #                 sink_point_id=sink
    #             )
    #
    #             for j in range(0, len(path), 2):
    #                 new_circle_points.append(path[j])
    #
    #             prev_close_bracket = close_bracket
    #
    #         for j in range(prev_close_bracket, n_circle_points):
    #             new_circle_points.append(all_circle_points[j])
    #
    #     else:
    #         open_bracket = open_brackets[-1]
    #         for i in range(n_brackets):
    #             close_bracket = close_brackets[i]
    #             next_open_bracket = open_brackets[i]
    #
    #             source = tooth_easy_mesh.find_closest_point(all_circle_points[open_bracket])
    #             sink = tooth_easy_mesh.find_closest_point(all_circle_points[close_bracket])
    #             path = tooth_easy_mesh.find_geodesic_path(
    #                 source_point_id=source,
    #                 sink_point_id=sink
    #             )
    #
    #             for j in range(0, len(path), 2):
    #                 new_circle_points.append(path[j])
    #
    #             for j in range(close_bracket, next_open_bracket):
    #                 new_circle_points.append(all_circle_points[j])
    #
    #             open_bracket = next_open_bracket
    #
    #     tooth.circle_points["list"] = new_circle_points
    #     tooth.circle_points["pcd"] = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(new_circle_points))

    def export_teeth_objects(
            self,
            step_count,
            teeth_steps_path,
            teeth_circle_steps_path,
            teeth_triangle_steps_path
    ):
        for tooth_idx in range(Constants.UPPER_IDX - Constants.LOWER_IDX):
            tooth: Tooth = self.teeth_objects[tooth_idx]

            if tooth.is_dummy:
                continue

            mesh = tooth.tooth
            mesh.compute_vertex_normals()

            tooth_step_name = f'step_{str(step_count).zfill(3)}_' \
                              f'Tooth_{str(Constants.LOWER_IDX + tooth_idx).zfill(3)}'

            o3d.io.write_triangle_mesh(
                filename=os.path.join(teeth_steps_path, f'{tooth_step_name}.stl'),
                mesh=mesh
            )

            if step_count == 0:
                print(f'{tooth_idx=}')
                print({
                    'tooth_crown_center': tooth.crown['center_point'],
                    'bone_center': tooth.bone_center,
                    'circle_points': tooth.snapping_points['list']
                })
            save_as_json(
                data={
                    'tooth_crown_center': tooth.crown['center_point'],
                    'bone_center': tooth.bone_center,
                    'circle_points': tooth.snapping_points['list']
                },
                file_name=os.path.join(teeth_circle_steps_path, f'{tooth_step_name}_circle.json')
            )

            if tooth.triangle is not None:
                triangle_mesh = tooth.triangle
                triangle_mesh.compute_vertex_normals()
                o3d.io.write_triangle_mesh(
                    filename=os.path.join(teeth_triangle_steps_path, f'{tooth_step_name}_triangle.stl'),
                    mesh=triangle_mesh
                )

    def save_front_point_logs(
            self,
            step_count,
            front_point_logs_path
    ):
        front_point_logs = []
        for tooth_idx in range(Constants.MAX_TEETH_COUNT):
            tooth: Tooth = self.teeth_objects[tooth_idx]
            if tooth.is_dummy:
                continue

            front_point_log = {
                'tooth_id': tooth_idx + 1,
                'front_point': convert_point_to_3d_team_format(
                    tooth.crown_front['surface_point']),
            }

            front_point_logs.append(front_point_log)

        json_obj = json.dumps(front_point_logs, indent=4)
        with open(os.path.join(front_point_logs_path, f'log{step_count}.json'), "w") as log_file:
            log_file.write(json_obj)

    def find_closest_neighbour_teeth(self, low=0, high=16, other_low=16, other_high=32):
        prev_not_missing_sphere: Union[Tooth, None] = None

        for tooth_idx in range(low, high):
            sphere: Tooth = self.teeth_objects[tooth_idx]
            if sphere.is_dummy:
                continue

            if prev_not_missing_sphere is None:
                sphere.same_jaw_neighbours.append(-1)
            else:
                prev_not_missing_sphere.same_jaw_neighbours.append(sphere.tooth_id-1)
                sphere.same_jaw_neighbours.append(prev_not_missing_sphere.tooth_id-1)

            prev_not_missing_sphere = sphere

            closest_other_jaw_tooth_idx = None
            second_closest_other_jaw_tooth_idx = None

            min_distance = 10000
            second_min_distance = 10000

            for other_jaw_neighbour_tooth_idx in range(other_low, other_high):
                neighbour_sphere: Tooth = self.teeth_objects[other_jaw_neighbour_tooth_idx]
                if neighbour_sphere.is_dummy:
                    continue

                # distance = sphere.vedo_check_collision(neighbour_sphere.decimated_point_cloud.points)
                distance = sphere.check_collision_with_point_cloud(neighbour_sphere)
                if distance < min_distance:
                    second_closest_other_jaw_tooth_idx = closest_other_jaw_tooth_idx
                    second_min_distance = min_distance

                    closest_other_jaw_tooth_idx = other_jaw_neighbour_tooth_idx
                    min_distance = distance

                elif distance < second_min_distance:
                    second_closest_other_jaw_tooth_idx = other_jaw_neighbour_tooth_idx
                    second_min_distance = distance

            assert closest_other_jaw_tooth_idx is not None and second_closest_other_jaw_tooth_idx is not None, \
                f'Could not find closest other jaw neighbours for Tooth_{tooth_idx+1}'

            sphere.other_jaw_neighbours += [closest_other_jaw_tooth_idx, second_closest_other_jaw_tooth_idx]

        prev_not_missing_sphere.same_jaw_neighbours.append(-1)

    def calculate_collision_distance(self):
        for tooth_idx in range(32):
            sphere: Tooth = self.teeth_objects[tooth_idx]
            if sphere.is_dummy:
                continue

            for neighbour_tooth_idx in sphere.same_jaw_neighbours + sphere.other_jaw_neighbours:
                if neighbour_tooth_idx == -1:
                    continue

                neighbour_sphere: Tooth = self.teeth_objects[neighbour_tooth_idx]
                if neighbour_sphere.is_dummy:
                    continue

                sphere.neighbouring_teeth_distance[neighbour_tooth_idx] = sphere.check_collision_with_point_cloud(
                    neighbour_sphere)
                # sphere.neighbouring_teeth_distance[neighbour_tooth_idx] = sphere.vedo_check_collision(
                #     neighbour_sphere.decimated_point_cloud.points)

    # def reorder_teeth_circle_points(
    #         self,
    # ):
    #     """
    #     Reordering circle points of the teeth such that the starting point is at the junction
    #     of the 2 neighbouring teeth
    #     """
    #     for r in [(0, 15), (16, 31)]:
    #         # looping over the maxillary/mandibular teeth except the last tooth in each of them
    #         prev_tooth_idx = -1
    #         current_tooth_idx = r[0]
    #         for tooth_idx in range(r[0] + 1, r[1]):
    #             current_tooth: Tooth = self.teeth_objects[current_tooth_idx]
    #             if current_tooth.is_dummy:
    #                 current_tooth_idx = tooth_idx
    #                 continue
    #
    #             neighbour_tooth: Tooth = self.teeth_objects[tooth_idx]
    #             if neighbour_tooth.is_dummy:
    #                 continue
    #
    #             neighbour_center_point = neighbour_tooth.crown['center_point']
    #             min_dist = 100000
    #             start_point_idx = -1
    #             for i, circle_point in enumerate(current_tooth.circle_points['list']):
    #                 dist = np.linalg.norm(circle_point - neighbour_center_point)
    #                 if dist < min_dist:
    #                     min_dist = dist
    #                     start_point_idx = i
    #
    #             current_tooth.reorder_circle_points(start_point_idx=start_point_idx)
    #
    #             prev_tooth_idx = current_tooth_idx
    #             current_tooth_idx = tooth_idx
    #
    #         assert prev_tooth_idx != -1, f'Some error occurred while reordering the circle points of the teeth for {r}'
    #
    #         # for the last not dummy/missing tooth, we are finding the circle point
    #         # at a maximum distance from the previous not dummy/missing tooth
    #         current_tooth: Tooth = self.teeth_objects[current_tooth_idx]
    #         prev_tooth: Tooth = self.teeth_objects[prev_tooth_idx]
    #
    #         neighbour_center_point = prev_tooth.crown['center_point']
    #         old_circle_points_list = current_tooth.circle_points['list']
    #         max_dist = 0
    #         start_point_idx = -1
    #         for i, circle_point in enumerate(old_circle_points_list):
    #             dist = np.linalg.norm(circle_point - neighbour_center_point)
    #             if dist > max_dist:
    #                 max_dist = dist
    #                 start_point_idx = i
    #
    #         current_tooth.reorder_circle_points(start_point_idx=start_point_idx)

    def check_collision_with_neighbours(
            self,
            tooth_idx: int,  # 0-based index,
    ):
        sphere: Tooth = self.teeth_objects[tooth_idx]
        final_sphere: Tooth = self.final_teeth.teeth_objects[tooth_idx]

        # checking collision with only same jaw neighbours
        for neighbour_tooth_idx in sphere.same_jaw_neighbours:
            if neighbour_tooth_idx == -1:
                continue

            neighbour_sphere: Tooth = self.teeth_objects[neighbour_tooth_idx]
            if neighbour_sphere.is_dummy:
                continue

            # distance = sphere.vedo_check_collision(neighbour_sphere.decimated_point_cloud.points)
            distance = sphere.check_collision_with_point_cloud(neighbour_sphere)
            # collision_distance = final_sphere.neighbouring_teeth_distance[neighbour_tooth_idx]
            if distance < Constants.SAME_JAW_COLLISION_THRESHOLD:
                return True

        if tooth_idx < 16:
            return False

        for neighbour_tooth_idx in sphere.other_jaw_neighbours:
            neighbour_sphere: Tooth = self.teeth_objects[neighbour_tooth_idx]
            if neighbour_sphere.is_dummy:
                continue

            # distance = sphere.vedo_check_collision(neighbour_sphere.decimated_point_cloud.points)
            distance = sphere.check_collision_with_point_cloud(neighbour_sphere)
            if distance < Constants.OTHER_JAW_COLLISION_THRESHOLD:
                return True

        return False

    def update_neighbouring_tooth_spacing(
            self,
            tooth_idx: int,  # 0-based index
    ):
        sphere: Tooth = self.teeth_objects[tooth_idx]

        for neighbour_tooth_idx, _ in sphere.neighbouring_teeth_distance.items():
            if neighbour_tooth_idx == -1:
                continue

            neighbour_sphere: Tooth = self.teeth_objects[neighbour_tooth_idx]
            # distance = sphere.vedo_check_collision(neighbour_sphere.decimated_point_cloud.points)
            distance = sphere.check_collision_with_point_cloud(neighbour_sphere)
            sphere.neighbouring_teeth_distance[neighbour_tooth_idx] = distance
            neighbour_sphere.neighbouring_teeth_distance[tooth_idx] = distance

    def save_teeth_stls(
            self,
            save_folder
    ):
        renew_folder(save_folder)
        renew_folder(f'{save_folder}points/')

        for i in range(len(self.teeth_objects)):
            sphere = self.teeth_objects[i]
            if sphere.is_dummy:
                continue

            o3d.io.write_triangle_mesh(
                filename=f'{save_folder}{sphere.name}.stl',
                mesh=sphere.tooth
            )

            left_point = sphere.crown_top["left_point"]
            right_point = sphere.crown_top["right_point"]
            center_point = sphere.crown_top["center_point"]
            front_surface_point = sphere.crown_front["surface_point"]

            data = {"left_point": {
                "_x": left_point[0],
                "_y": left_point[2],
                "_z": left_point[1],
            }, "right_point": {
                "_x": right_point[0],
                "_y": right_point[2],
                "_z": right_point[1],
            }, "center_point": {
                "_x": center_point[0],
                "_y": center_point[2],
                "_z": center_point[1],
            }, "front_surface_point": {
                "_x": front_surface_point[0],
                "_y": front_surface_point[2],
                "_z": front_surface_point[1],
            }}

            save_as_json(data, file_name=f'{save_folder}points/{sphere.name}_points.json')

    def draw_teeth_with_points_picked(
            self,
            final_teeth=None,
            low: int = 0,
            high: int = 32,
            show_teeth: bool = True
    ):
        lines_points = []
        for i, sphere in enumerate(self.teeth_objects):
            if not low <= i < high or sphere.crown_top is None:
                continue

            lines_points.append(
                draw_line_end_points([sphere.crown_front["surface_point"]], color=[1, 0, 0]))
            lines_points.append(draw_line_end_points([sphere.crown_top["left_point"]], color=[0, 1, 0]))
            lines_points.append(draw_line_end_points([sphere.crown_top["right_point"]], color=[0, 0, 1]))

            lines_points.append(get_line_segment_between_points(sphere.crown_top["left_point"],
                                                                sphere.crown_top["right_point"])),
            lines_points.append(get_line_segment_between_points(sphere.crown_top["right_point"],
                                                                sphere.crown_front["surface_point"])),
            lines_points.append(get_line_segment_between_points(sphere.crown_front["surface_point"],
                                                                sphere.crown_top["left_point"])),

        if final_teeth is not None:
            for i, sphere in enumerate(final_teeth.teeth_objects):
                if not low <= i < high or sphere.crown_top is None:
                    continue

                lines_points.append(
                    draw_line_end_points([sphere.crown_front["surface_point"]], color=[1, 0, 1]))
                lines_points.append(draw_line_end_points([sphere.crown_top["left_point"]], color=[0, 0.5, 0]))
                lines_points.append(draw_line_end_points([sphere.crown_top["right_point"]], color=[0, 1, 1]))

                lines_points.append(get_line_segment_between_points(sphere.crown_top["left_point"],
                                                                    sphere.crown_top["right_point"],
                                                                    color=[0, 1, 1])),
                lines_points.append(get_line_segment_between_points(sphere.crown_top["right_point"],
                                                                    sphere.crown_front["surface_point"],
                                                                    color=[0, 1, 1])),
                lines_points.append(get_line_segment_between_points(sphere.crown_front["surface_point"],
                                                                    sphere.crown_top["left_point"],
                                                                    color=[0, 1, 1])),

        c_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0)
        lines_points.append(c_mesh)
        if show_teeth:
            draw_objects_for_demo(self.teeth_objects[low:high], lines=lines_points)
        else:
            draw_objects_for_demo([], lines=lines_points)

    def draw_teeth_with_axes(
            self,
            low: int = 0,
            high: int = 32,
    ):
        objects = []
        for tooth_idx in range(low, high):
            sphere: Tooth = self.teeth_objects[tooth_idx]
            if sphere.is_dummy:
                continue

            objects.append({'name': f'tooth_{tooth_idx}', 'geometry': sphere.tooth})
            objects.append({'name': f'top_{tooth_idx}', 'geometry': get_line_segment_between_points(
                point1=sphere.top_axis.first_point,
                point2=sphere.top_axis.second_point,
                color=(0, 0, 1),
            )})

            objects.append({'name': f'side_{tooth_idx}', 'geometry': get_line_segment_between_points(
                point1=sphere.side_axis.first_point,
                point2=sphere.side_axis.second_point,
                color=(1, 0, 0),
            )})

            objects.append({
                'name': f'front_{tooth_idx}',
                'geometry': get_line_segment_between_points(
                    point1=sphere.front_axis.first_point,
                    point2=sphere.front_axis.second_point,
                    color=(0, 1, 0),
                )
            })

        o3d.visualization.draw(
            objects,
            bg_color=(0.0, 0.0, 0.0, 1.0),
            show_skybox=False
        )

    def draw_teeth(
            self,
            low: int = 0,
            high: int = 32,
            show_dummy: bool = False
    ):
        objects = []
        for i, sphere in enumerate(self.teeth_objects):
            if sphere.is_dummy:
                continue

            if low <= i < high and (show_dummy or not sphere.is_dummy):
                objects.append(sphere.tooth)

        o3d.visualization.draw_geometries(
            objects
        )

    def to_json(self, file_name: str):
        data = dict()
        for tooth_idx in range(len(self.teeth_objects)):
            sphere: Tooth = self.teeth_objects[tooth_idx]
            tooth_id = str(tooth_idx + 1)

            if sphere.is_dummy:
                data[tooth_id] = {
                    "is_dummy": True,
                    "crown": {},
                    "crown_top": {},
                    "crown_front": {},
                    "center_edges": {},
                    "top_edges": {},
                    "front_edges": {},
                    "top_axis": {},
                    "front_axis": {},
                    "side_axis": {},
                }
                continue

            data[tooth_id] = {
                "is_dummy": False,
                "crown": {
                    "center_point": sphere.crown["center_point"],
                },
                "crown_top": {
                    "center_point": sphere.crown_top["center_point"],
                    "left_point": sphere.crown_top["left_point"],
                    "right_point": sphere.crown_top["right_point"],
                    "crown_slope": sphere.crown_top["crown_slope"],
                    "center_slope": sphere.crown_top["center_slope"],
                },
                "crown_front": {
                    "surface_point": sphere.crown_front["surface_point"],
                    "surface_point_slope": sphere.crown_front["surface_point_slope"],
                },
                "top_axis": sphere.top_axis.to_dict(),
                "front_axis": sphere.front_axis.to_dict(),
                "side_axis": sphere.side_axis.to_dict(),
                'center_edges': {},
                'top_edges': {},
                'front_edges': {},
            }

            for ni, neighbours in enumerate([sphere.other_jaw_neighbours, sphere.same_jaw_neighbours]):
                other_jaw = (ni == 0)
                for neighbour_tooth_idx in neighbours:
                    if neighbour_tooth_idx == -1:
                        continue

                    neighbour_tooth_id = neighbour_tooth_idx + 1
                    neighbour_sphere: Tooth = self.teeth_objects[neighbour_tooth_idx]
                    if neighbour_sphere.is_dummy:
                        continue

                    data[tooth_id]['center_edges'][str(neighbour_tooth_idx)] = {
                        'other_jaw': other_jaw,
                        'slope': get_slope([
                            sphere.crown['center_point'],
                            neighbour_sphere.crown['center_point'],
                        ]),
                        'length': np.linalg.norm(
                            sphere.crown['center_point'] - neighbour_sphere.crown['center_point']
                        ),
                    }

                    data[tooth_id]['top_edges'][str(neighbour_tooth_id)] = {
                        'other_jaw': other_jaw,
                        'slope': get_slope([
                            sphere.crown_top['center_point'],
                            neighbour_sphere.crown_top['center_point'],
                        ]),
                        'length': np.linalg.norm(
                            sphere.crown_top['center_point'] - neighbour_sphere.crown_top['center_point']
                        ),
                    }

                    data[tooth_id]['front_edges'][str(neighbour_tooth_id)] = {
                        'other_jaw': other_jaw,
                        'slope': get_slope([
                            sphere.crown_front['surface_point'],
                            neighbour_sphere.crown_front['surface_point'],
                        ]),
                        'length': np.linalg.norm(
                            sphere.crown_front['surface_point'] - neighbour_sphere.crown_front['surface_point']
                        ),
                    }

        save_as_json(data, file_name)

    def tooth_step_translate(
            self,
            tooth_idx: int,
            translation_info,
            check_collision: bool = True,
            reasonable_torque: bool = False,
            misc_check_passed: bool = True,
    ) -> bool:
        translation_info['step_movement'] = 0

        if translation_info['done']:
            return False

        if tooth_idx not in Constants.MOLAR_RANGE and not reasonable_torque:
            return False

        if not misc_check_passed:
            return False

        initial_sphere: Tooth = self.teeth_objects[tooth_idx]
        initial_sphere_copy = copy.deepcopy(initial_sphere)

        translation_done = False
        distance_left = translation_info['distance_left']
        distance_to_translate = Constants.DISTANCE_MOVEMENT_THRESHOLD if distance_left > 0 \
            else -Constants.DISTANCE_MOVEMENT_THRESHOLD

        if abs(distance_left) < abs(distance_to_translate):
            distance_to_translate = distance_left
            translation_done = True

        movement = distance_to_translate * translation_info['slope']
        initial_sphere.translate_coordinates(
            x_dist=movement[0],
            y_dist=movement[1],
            z_dist=movement[2],
        )

        if check_collision and self.check_collision_with_neighbours(tooth_idx=tooth_idx):
            self.teeth_objects[tooth_idx] = initial_sphere_copy
            return False

        translation_info['step_movement'] = np.round(distance_to_translate, Constants.DECIMAL_PRECISION)
        translation_info['distance_translated'] += distance_to_translate
        translation_info['distance_left'] -= distance_to_translate
        translation_info['done'] = translation_done

        self.update_neighbouring_tooth_spacing(tooth_idx=tooth_idx)

        return True

    def tooth_step_rotate(
            self,
            tooth_idx: int,
            rotation_info,
            positive_angle_threshold: float,
    ):
        rotation_info['step_movement'] = 0

        if rotation_info['done']:
            return False, False

        initial_sphere: Tooth = self.teeth_objects[tooth_idx]
        initial_sphere_copy = copy.deepcopy(initial_sphere)

        rotation_done = False
        angle_left = rotation_info['angle_left']
        rotation_angle = positive_angle_threshold if angle_left > 0 else -positive_angle_threshold
        if abs(angle_left) <= positive_angle_threshold:
            rotation_done = True
            rotation_angle = angle_left

        initial_sphere.rotate_using_quaternion(
            axis=rotation_info['axis'],
            angle=rotation_angle,
            rotating_center=initial_sphere.crown['center_point'],
            move_back=True,
        )

        # if self.check_collision_with_neighbours(tooth_idx=tooth_idx):
        #     self.teeth_objects[tooth_idx] = initial_sphere_copy
        #     return False, True

        rotation_info['step_movement'] = np.round(np.rad2deg(rotation_angle), Constants.DECIMAL_PRECISION)
        rotation_info['angle_rotated'] += rotation_angle
        rotation_info['angle_left'] -= rotation_angle
        rotation_info['done'] = rotation_done

        self.update_neighbouring_tooth_spacing(tooth_idx=tooth_idx)

        return True, False

    def get_teeth_spacing(
            self,
            movement_info
    ):
        teeth_spacing = []
        for tooth_idx in range(32):
            sphere: Tooth = self.teeth_objects[tooth_idx]
            tooth_movement_info: MovementInfo = movement_info[tooth_idx]

            if sphere.is_dummy or tooth_movement_info.done:
                continue

            if tooth_movement_info.translation_side['distance_left'] >= 0:
                same_jaw_space = sphere.neighbouring_teeth_distance[sphere.same_jaw_neighbours[-1]]
            else:
                same_jaw_space = sphere.neighbouring_teeth_distance[sphere.same_jaw_neighbours[0]]

            # same_jaw_space = 0
            # for neighbour_tooth_idx in sphere.same_jaw_neighbours:
            #     same_jaw_space += sphere.neighbouring_teeth_distance[neighbour_tooth_idx]

            other_jaw_space = 0
            for neighbour_tooth_idx in sphere.other_jaw_neighbours:
                other_jaw_space += sphere.neighbouring_teeth_distance[neighbour_tooth_idx]

            teeth_spacing.append((-same_jaw_space, -other_jaw_space, tooth_idx))

        heapq.heapify(teeth_spacing)

        return teeth_spacing

