from . import Tooth
from .ToothEasyMesh import ToothEasyMesh
from .functions import *


class Teeth:
    def __init__(
            self,
            case_folder,
            circles_folder="",
            model_folder="",
            is_final=False,
    ):
        self.maxillary_count = 16
        self.mandibular_count = 16
        self.teeth_objects = []
        self.case_folder = case_folder
        self.circles_folder = circles_folder
        self.model_folder = model_folder
        self.is_final = is_final

        self.import_teeth_objects()

    def import_teeth_objects(self):
        for tooth_id in range(Constants.LOWER_IDX, Constants.UPPER_IDX):
            tooth_idx = tooth_id - 1
            tooth = Tooth(
                tooth_id=tooth_id,
                file_name=Constants.TEETH_FILE_NAME,
                case_folder=self.case_folder,
                circles_folder=self.circles_folder,
                model_folder=self.model_folder,
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

        for tooth_idx in range(Constants.MAX_TEETH_COUNT):
            tooth: Tooth = self.teeth_objects[tooth_idx]
            if tooth.is_dummy:
                continue

            if tooth_idx in Constants.INCISORS_RANGE or tooth_idx in Constants.PREMOLARS_RANGE:
                self.remove_high_elevation_points_of_segmentation_line(tooth_idx=tooth_idx)

            tooth.get_snapping_points_on_the_tooth_surface()

    def remove_high_elevation_points_of_segmentation_line(
            self,
            tooth_idx,
    ):
        tooth: Tooth = self.teeth_objects[tooth_idx]
        if tooth.is_dummy:
            return

        all_circle_points = tooth.circle_points["list"]
        n_circle_points = len(all_circle_points)
        ncp = []

        # remove the points which are touching the circle points of neighbouring teeth
        is_gum_points = [True] * n_circle_points
        for neighbour_tooth_idx in tooth.same_jaw_neighbours:
            if neighbour_tooth_idx == -1:
                continue

            neighbour_tooth: Tooth = self.teeth_objects[neighbour_tooth_idx]
            if neighbour_tooth.is_dummy:
                continue

            neighbour_circle_points = neighbour_tooth.circle_points["original_list"]
            ncp += list(neighbour_circle_points)

            signed_distances = self.check_collision_with_neighbour_circle_points(
                tooth=neighbour_tooth,
                circle_points=all_circle_points
            )

            for curr_i in range(n_circle_points):
                # circle_point = all_circle_points[curr_i]

                # calculate the minimum distance of circle_point from the neighbour circle points using numpy
                # min_distance = np.min(np.linalg.norm(neighbour_circle_points - circle_point, axis=1))
                min_distance = signed_distances[curr_i]

                if min_distance < Constants.GUMLINE_NEIGHBOURING_DISTANCE_THRESHOLD:
                    is_gum_points[curr_i] = not is_gum_points[curr_i]
                    tooth.closest_points.append(all_circle_points[curr_i])

        gum_points = []
        not_gum_points = []
        for i in range(n_circle_points):
            if is_gum_points[i]:
                gum_points.append(all_circle_points[i])
            else:
                not_gum_points.append(all_circle_points[i])

        if len(not_gum_points) < 5:
            return

        # find the open and close brackets
        open_brackets = []
        close_brackets = []
        for i in range(n_circle_points):
            prev_i = i
            curr_i = (i + 1) % n_circle_points

            if is_gum_points[prev_i] and not is_gum_points[curr_i]:
                open_brackets.append(prev_i)
            elif not is_gum_points[prev_i] and is_gum_points[curr_i]:
                close_brackets.append(prev_i)

        tooth_easy_mesh: ToothEasyMesh = tooth.tooth_easy_mesh

        n_brackets = len(open_brackets)
        new_circle_points = []

        if open_brackets[0] < close_brackets[0]:
            prev_close_bracket = 0
            for i in range(n_brackets):
                open_bracket = open_brackets[i]
                close_bracket = close_brackets[i]

                for j in range(prev_close_bracket, open_bracket):
                    new_circle_points.append(all_circle_points[j])

                source = tooth_easy_mesh.find_closest_point(all_circle_points[open_bracket])
                sink = tooth_easy_mesh.find_closest_point(all_circle_points[close_bracket])
                path = tooth_easy_mesh.find_geodesic_path(
                    source_point_id=source,
                    sink_point_id=sink
                )

                for j in range(0, len(path)):
                    new_circle_points.append(path[j])

                prev_close_bracket = close_bracket

            for j in range(prev_close_bracket, n_circle_points):
                new_circle_points.append(all_circle_points[j])

        else:
            open_bracket = open_brackets[-1]
            for i in range(n_brackets):
                close_bracket = close_brackets[i]
                next_open_bracket = open_brackets[i]

                source = tooth_easy_mesh.find_closest_point(all_circle_points[open_bracket])
                sink = tooth_easy_mesh.find_closest_point(all_circle_points[close_bracket])
                path = tooth_easy_mesh.find_geodesic_path(
                    source_point_id=source,
                    sink_point_id=sink
                )

                for j in range(0, len(path)):
                    new_circle_points.append(path[j])

                for j in range(close_bracket, next_open_bracket):
                    new_circle_points.append(all_circle_points[j])

                open_bracket = next_open_bracket

        tooth.circle_points["list"] = new_circle_points
        tooth.circle_points["pcd"] = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(new_circle_points))

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

    def check_collision_with_neighbours(
            self,
            tooth_idx: int,  # 0-based index,
    ):
        sphere: Tooth = self.teeth_objects[tooth_idx]

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

    @staticmethod
    def check_collision_with_neighbour_circle_points(
            tooth: Tooth,
            circle_points: np.ndarray,
    ):
        scene = o3d.t.geometry.RaycastingScene()
        mesh = copy.deepcopy(tooth.tooth)
        mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

        _ = scene.add_triangles(mesh)

        query_points = o3d.core.Tensor(np.asarray(circle_points), dtype=o3d.core.Dtype.Float32)

        signed_distance = scene.compute_signed_distance(query_points)

        return signed_distance

    def export_teeth_circle_points(
            self,
            teeth_circle_steps_path,
    ):
        for tooth_idx in range(Constants.UPPER_IDX - Constants.LOWER_IDX):
            tooth: Tooth = self.teeth_objects[tooth_idx]

            if tooth.is_dummy:
                continue

            mesh = tooth.tooth
            mesh.compute_vertex_normals()

            save_as_json(
                data={
                    'circle_points': tooth.snapping_points['list']
                },
                file_name=os.path.join(teeth_circle_steps_path, f'Tooth_{str(tooth_idx + 1)}_circle.json')
            )
