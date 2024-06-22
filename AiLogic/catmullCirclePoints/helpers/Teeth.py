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

            tooth.get_snapping_points_on_the_tooth_surface()

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

            o3d.io.write_point_cloud(
                os.path.join(teeth_circle_steps_path, f'Tooth_{str(tooth_idx + 1)}_snapping.ply'),
                tooth.snapping_points['pcd']
            )
