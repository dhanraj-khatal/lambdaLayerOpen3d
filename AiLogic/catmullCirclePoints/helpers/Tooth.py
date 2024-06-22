from .ToothEasyMesh import ToothEasyMesh
from .functions import *

_logger = logging.getLogger()


class Tooth:
    def __init__(
            self,
            file_name: str,
            tooth_id: int,
            case_folder: str,
            circles_folder: str = "",
            model_folder: str = "",
            is_final: bool = False
    ):
        self.tooth_id = tooth_id
        self.name = file_name + str(self.tooth_id)
        self.is_tooth_part_of_maxillary = self.tooth_id < 17

        self.is_dummy = False
        self.done = False

        self.case_folder = case_folder
        self.circles_folder = circles_folder
        self.model_folder = model_folder

        self.is_final = is_final

        tooth_stl_filepath = os.path.join(self.case_folder, f'{self.name}.stl')
        self.original_triangle = o3d.geometry.TriangleMesh.create_sphere(radius=2.0)
        self.tooth_easy_mesh = None

        if os.path.isfile(tooth_stl_filepath):
            self.original_tooth = o3d.io.read_triangle_mesh(tooth_stl_filepath)
            self.tooth_easy_mesh: ToothEasyMesh = ToothEasyMesh(tooth_stl_filepath)

        else:
            _logger.warning(f"{self.name} is missing")
            self.is_dummy = True
            self.done = True
            self.original_tooth = o3d.geometry.TriangleMesh.create_sphere(radius=2.0)

        # below 2 lists should contain the tooth ids of the nearest neighbours (1-based index)
        self.same_jaw_neighbours = []
        self.other_jaw_neighbours = []

        self.neighbouring_teeth_distance = {
            -1: 10000
        }

        self.tooth: Union[o3d.geometry.TriangleMesh, None] = None

        # decimated mesh
        self.decimated_point_cloud = None

        # 12 circle points
        self.circle_points = None

        self.rig_points = []

        self.snapping_points = None

        self.closest_points = []

        self.reset_tooth()

    # checked
    def reset_tooth(self):
        self.tooth = copy.deepcopy(self.original_tooth)
        self.tooth.compute_vertex_normals()
        self.tooth.paint_uniform_color([0.7, 0.7, 0.7])

        tooth = copy.deepcopy(self.original_tooth)
        decimated_mesh = tooth.merge_close_vertices(0.2)
        self.decimated_point_cloud = decimated_mesh.sample_points_poisson_disk(1000)

        if not self.is_dummy:
            self.get_circle_points_on_the_tooth_surface_through_tool()

    def get_circle_points_on_the_tooth_surface_through_tool(self):
        circle_file_path = os.path.join(self.circles_folder, f'{self.name}_gum_points.json')
        if not os.path.isfile(circle_file_path):
            assert self.is_dummy, f'File missing: {circle_file_path}'
            return

        all_circle_points = get_json_object(circle_file_path)

        circle_points = []
        added_points = set()
        for circle_point in all_circle_points:
            point_id = self.tooth_easy_mesh.find_closest_point(circle_point)
            if point_id not in added_points:
                circle_points.append(self.tooth_easy_mesh.get_point(point_id))

        circle_points = smoothen_circle_points(circle_points)

        self.circle_points = {
            "original_list": circle_points,
            "list": circle_points,
            "pcd": o3d.geometry.PointCloud(o3d.utility.Vector3dVector(circle_points)),
        }

    def get_snapping_points_on_the_tooth_surface(
            self,
    ):
        if self.is_dummy:
            return

        all_circle_points = self.circle_points["list"]

        print(f'{self.tooth_id=} {all_circle_points=}')
        _logger.info(f'{self.tooth_id=} {all_circle_points=}')

        # get equidistant points on the segmentation line
        snapping_points = get_equidistant_circle_points(
            points=all_circle_points, no_of_equidistant_points_required=20)

        print(f'{self.tooth_id=} get_equidistant_circle_points {snapping_points=}')

        rigged_gum_points_filepath = os.path.join(
            self.model_folder, f'rig_points', f'bonePosData_{self.tooth_id}.json')

        rig_file_present = os.path.isfile(rigged_gum_points_filepath)

        print(f'{self.tooth_id=} {rig_file_present=}')
        _logger.info(f'{self.tooth_id=} {rig_file_present=}')

        # apply snapping points on all the teeth
        if rig_file_present:
            rig_points = get_json_object(rigged_gum_points_filepath)
            rig_points = [np.array([point[0], point[2], point[1]]) for point in rig_points]
            self.rig_points = rig_points

            print(f'{self.tooth_id=} {self.rig_points=}')
            _logger.info(f'{self.tooth_id=} {self.rig_points=}')

            snapping_points_center_point = get_avg_point(points=snapping_points)
            rig_points_center_point = get_avg_point(points=rig_points)

            print(f'{self.tooth_id=} {snapping_points_center_point=}')
            print(f'{self.tooth_id=} {rig_points_center_point=}')

            movement = snapping_points_center_point - rig_points_center_point
            rig_points = move_points(
                points=rig_points,
                x=movement[0],
                y=movement[1],
                z=movement[2]
            )

            reordered_snapping_points_list = []
            n_points = len(snapping_points)
            idx = np.argmin(np.linalg.norm(np.asarray(snapping_points) - rig_points[0], axis=1))
            for i in range(n_points):
                reordered_snapping_points_list.append(snapping_points[idx])
                idx = (idx + 1) % n_points

            snapping_points = reordered_snapping_points_list
            print(f'{self.tooth_id=} {snapping_points=}')
            _logger.info(f'{self.tooth_id=} {snapping_points=}')

        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(snapping_points))
        pcd.paint_uniform_color([0, 1, 0])
        self.snapping_points = {
            "list": snapping_points,
            "pcd": pcd,
        }

    def update_snapping_points(self):
        new_snapping_points = []
        for point in np.asarray(self.snapping_points["pcd"].points):
            new_snapping_points.append(list(point))

        self.snapping_points["list"] = new_snapping_points

    # section: miscellaneous functions using movement functions and other open3d internal functions

    # To detect the collision between teeth using Raycasting operation in open3d
    def check_collision_with_point_cloud(
            self,
            sphere,
    ):
        scene = o3d.t.geometry.RaycastingScene()
        mesh = copy.deepcopy(self.tooth)
        mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

        _ = scene.add_triangles(mesh)

        query_points = o3d.core.Tensor(np.asarray(sphere.decimated_point_cloud.points), dtype=o3d.core.Dtype.Float32)

        signed_distance = scene.compute_signed_distance(query_points)
        minimum_distance = min(signed_distance.numpy())

        return minimum_distance
