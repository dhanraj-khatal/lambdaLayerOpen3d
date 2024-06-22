from copy import deepcopy

# import vedo
from scipy.spatial.transform import Rotation

from .ToothAxis import ToothAxis
from .ToothEasyMesh import ToothEasyMesh
from .functions import *

_logger = logging.getLogger()


class Tooth:
    def __init__(
            self,
            file_name: str,
            tooth_id: int,
            config_parser,
            case_folder: str,
            points_folder: str,
            triangles_folder: str = "",
            circles_folder: str = "",
            axes_folder: str = "",
            model_folder: str = "",
            is_final: bool = False
    ):
        self.tooth_id = tooth_id
        self.name = file_name + str(self.tooth_id)
        self.is_tooth_part_of_maxillary = self.tooth_id < 17

        self.config_parser = config_parser

        self.is_dummy = False
        self.done = False

        self.case_folder = case_folder
        self.points_folder = points_folder
        self.triangles_folder = triangles_folder
        self.circles_folder = circles_folder
        self.axes_folder = axes_folder
        self.model_folder = model_folder

        self.is_final = is_final

        tooth_stl_filepath = os.path.join(self.case_folder, f'{self.name}.stl')
        self.original_triangle = o3d.geometry.TriangleMesh.create_sphere(radius=2.0)
        self.tooth_easy_mesh = None

        if os.path.isfile(tooth_stl_filepath):
            self.original_tooth = o3d.io.read_triangle_mesh(tooth_stl_filepath)
            # self.vedo_tooth = vedo.load(tooth_stl_filepath)

            if not is_final:
                self.tooth_easy_mesh: ToothEasyMesh = ToothEasyMesh(tooth_stl_filepath)
                if self.triangles_folder != "":
                    self.original_triangle = o3d.io.read_triangle_mesh(
                        os.path.join(self.triangles_folder, f'{self.name}_triangle.stl'))
        else:
            _logger.warning(f"{self.name} is missing")
            self.is_dummy = True
            self.done = True
            self.original_tooth = o3d.geometry.TriangleMesh.create_sphere(radius=2.0)
            self.original_triangle = o3d.geometry.TriangleMesh.create_sphere(radius=2.0)
            # self.vedo_tooth = vedo.Sphere(r=2.0)

        # not used
        # self.crown_point_threshold = float(self.config_parser.get(self.name, "CrownPointThreshold"))

        # self.same_jaw_neighbours = np.asarray(config_parser.get(self.name, "SameJawNeighbours").split()) \
        #     .astype("int")
        # self.other_jaw_neighbours = np.asarray(config_parser.get(self.name, "OtherJawNeighbours").split()) \
        #     .astype("int")

        # below 2 lists should contain the tooth ids of the nearest neighbours (1-based index)
        self.same_jaw_neighbours = []
        self.other_jaw_neighbours = []

        self.neighbouring_teeth_distance = {
            -1: 10000
        }

        self.tooth: Union[o3d.geometry.TriangleMesh, None] = None
        self.triangle: Union[o3d.geometry.TriangleMesh, None] = None

        # average center of the circles of a tooth, obtained from the axes api
        self.bone_center = None

        # crown info
        self.crown = None

        # crown top info
        self.crown_top = None

        # crown front info
        self.crown_front = None

        # decimated mesh
        self.decimated_point_cloud = None

        # 12 circle points
        self.circle_points = None

        # front axis
        self.front_axis = None

        # top axis
        self.top_axis = None

        # side axis
        self.side_axis = None

        self.pcd_tooth_axes = None

        self.rig_points = []

        self.snapping_points = None

        self.reset_tooth()

    # def __deepcopy__(self, memo):
    #     cls = self.__class__
    #     result = cls.__new__(cls)
    #     memo[id(self)] = result
    #     for k, v in self.__dict__.items():
    #         if k == 'vedo_tooth':
    #             setattr(result, k, self.vedo_tooth.clone(deep=True))
    #             continue
    #
    #         setattr(result, k, deepcopy(v, memo))
    #
    #     return result

    # checked
    def reset_tooth(self):
        self.tooth = copy.deepcopy(self.original_tooth)
        self.tooth.compute_vertex_normals()
        self.tooth.paint_uniform_color([0.7, 0.7, 0.7])

        self.triangle = copy.deepcopy(self.original_triangle)

        tooth = copy.deepcopy(self.original_tooth)
        decimated_mesh = tooth.merge_close_vertices(0.2)
        self.decimated_point_cloud = decimated_mesh.sample_points_poisson_disk(1000)
        self.get_tooth_axes_through_tool()

        if not self.is_dummy:
            if self.is_final:
                self.get_points_from_final_json()
            else:
                self.get_points_on_the_tooth_surface_through_tool()
                self.get_circle_points_on_the_tooth_surface_through_tool()

            self.done = False

    def get_tooth_axes_through_tool(self):
        filepath = os.path.join(self.axes_folder, f'{self.name}_axes.json')
        if self.is_final:
            filepath = os.path.join(self.axes_folder, f'{self.name}_axes_points.json')

        is_file_present = os.path.isfile(filepath)
        if not is_file_present:
            assert self.is_dummy, f'ERROR: File missing: {filepath}'
            return

        # Sample axes file
        # axes_picked = {
        #     "tooth_crown_center": {
        #         "_x": 20.545665006090136,
        #         "_y": 1.6935829667084437,
        #         "_z": 21.987026739005326
        #     },
        #     "bone_center": {
        #         "_x": 20.755973155681904,
        #         "_y": 3.793905746478301,
        #         "_z": 21.622198343276978
        #     },
        #     "top_point": {
        #         "_x": 20.59924442646697,
        #         "_y": 0.1496541459662497,
        #         "_z": 20.185844709554438
        #     },
        #     "front_point": {
        #         "_x": 16.73345826883028,
        #         "_y": 4.810334229876316,
        #         "_z": 26.481742838751096
        #     },
        #     "side_point": {
        #         "_x": 16.215426575222864,
        #         "_y": 5.621190632116232,
        #         "_z": 17.481538526677927
        #     },
        #     "top_axis": {
        #         "_x": -0.03997733646644528,
        #         "_y": -0.9295518003100711,
        #         "_z": -0.3663756656980257
        #     },
        #     "front_axis": {
        #         "_x": -0.629511678399854,
        #         "_y": 0.15906805035048824,
        #         "_z": 0.7605043356344262
        #     },
        #     "side_axis": {
        #         "_x": -0.6486495114941483,
        #         "_y": 0.26104069794827583,
        #         "_z": -0.5915228309427217
        #     }
        # }

        axes_picked = get_json_object(filepath)
        crown_center = swap_z_and_y_coordinates(axes_picked["tooth_crown_center"])
        top_point = swap_z_and_y_coordinates(axes_picked["top_point"])
        front_point = swap_z_and_y_coordinates(axes_picked["front_point"])
        side_point = swap_z_and_y_coordinates(axes_picked["side_point"])
        bone_center = swap_z_and_y_coordinates(axes_picked["bone_center"])

        self.crown = {
            "pcd_center_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([crown_center]))
        }

        self.update_crown_center()

        self.pcd_tooth_axes = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
            crown_center,
            top_point,
            front_point,
            side_point,
            bone_center,
        ]))

        self.bone_center = np.asarray(bone_center)
        self.top_axis: ToothAxis = ToothAxis(first_point=crown_center, second_point=top_point)
        self.front_axis: ToothAxis = ToothAxis(first_point=crown_center, second_point=front_point)
        self.side_axis: ToothAxis = ToothAxis(first_point=crown_center, second_point=side_point)

    def get_points_on_the_tooth_surface_through_tool(self):
        points_filepath = os.path.join(self.points_folder, f'{self.name}_points.json')
        if not os.path.isfile(points_filepath):
            assert self.is_dummy, f'ERROR: File missing: {points_filepath}'
            return

        points_picked = get_json_object(points_filepath)

        points = [
            swap_z_and_y_coordinates(points_picked["top_point1"]),
            swap_z_and_y_coordinates(points_picked["top_point2"]),
        ]

        self.crown_top = {
            "pcd_left_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                points[0]
            ])),
            "pcd_right_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                points[1]
            ])),
            "pcd_center_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                (np.asarray(points[0]) + np.asarray(points[1])) / 2
            ])),
        }

        self.update_crown_top_point()

        self.crown_front = {
            "pcd_surface_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                swap_z_and_y_coordinates(points_picked["front_point"]),
            ])),
        }

        self.update_crown_front_point()

    def get_circle_points_on_the_tooth_surface_through_tool(self):
        circle_file_path = os.path.join(self.circles_folder, f'{self.name}_circle.json')
        if not os.path.isfile(circle_file_path):
            assert self.is_dummy, f'File missing: {circle_file_path}'
            return

        circle_points_data = get_json_object(circle_file_path)
        circle_points = circle_points_data['circle_points']

        self.snapping_points = {
            "list": circle_points,
            "pcd": o3d.geometry.PointCloud(o3d.utility.Vector3dVector(circle_points)),
        }

    # def get_snapping_points_on_the_tooth_surface(
    #         self,
    # ):
    #     if self.is_dummy:
    #         return
    #
    #     all_circle_points = self.circle_points["list"]
    #     print(f'{self.tooth_id=} {all_circle_points=}')
    #     _logger.info(f'{self.tooth_id=} {all_circle_points=}')
    #     # get 12 equidistant points on the segmentation line
    #     snapping_points = get_equidistant_circle_points(
    #         points=all_circle_points, no_of_equidistant_points_required=20)
    #
    #     print(f'{self.tooth_id=} get_equidistant_circle_points {snapping_points=}')
    #
    #     rigged_gum_points_filepath = os.path.join(
    #         self.model_folder, f'rig_points', f'bonePosData_{self.tooth_id}.json')
    #
    #     rig_file_present = os.path.isfile(rigged_gum_points_filepath)
    #     print(f'{self.tooth_id=} {rig_file_present=}')
    #     _logger.info(f'{self.tooth_id=} {rig_file_present=}')
    #
    #     # apply snapping points on all the teeth
    #     if rig_file_present:
    #         rig_points = get_json_object(rigged_gum_points_filepath)
    #         rig_points = [np.array([point[0], point[2], point[1]]) for point in rig_points]
    #         self.rig_points = rig_points
    #         print(f'{self.tooth_id=} {self.rig_points=}')
    #         _logger.info(f'{self.tooth_id=} {self.rig_points=}')
    #
    #         snapping_points_center_point = get_avg_point(points=snapping_points)
    #         rig_points_center_point = get_avg_point(points=rig_points)
    #
    #         print(f'{self.tooth_id=} {snapping_points_center_point=}')
    #         print(f'{self.tooth_id=} {rig_points_center_point=}')
    #
    #         movement = snapping_points_center_point - rig_points_center_point
    #         rig_points = move_points(
    #             points=rig_points,
    #             x=movement[0],
    #             y=movement[1],
    #             z=movement[2]
    #         )
    #
    #         reordered_snapping_points_list = []
    #         n_points = len(snapping_points)
    #         idx = np.argmin(np.linalg.norm(np.asarray(snapping_points) - rig_points[0], axis=1))
    #         for i in range(n_points):
    #             reordered_snapping_points_list.append(snapping_points[idx])
    #             idx = (idx + 1) % n_points
    #
    #         snapping_points = reordered_snapping_points_list
    #         print(f'{self.tooth_id=} {snapping_points=}')
    #         _logger.info(f'{self.tooth_id=} {snapping_points=}')
    #     pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(snapping_points))
    #     pcd.paint_uniform_color([0, 1, 0])
    #     self.snapping_points = {
    #         "list": snapping_points,
    #         "pcd": pcd,
    #     }

    def get_points_from_final_json(self):
        points_filepath = os.path.join(self.points_folder, f'{self.name}_points.json')

        if not os.path.isfile(points_filepath):
            assert self.is_dummy, f'ERROR: File missing: {points_filepath}'
            return

        points_picked = get_json_object(points_filepath)

        points = [
            swap_z_and_y_coordinates(points_picked["left_point"]),
            swap_z_and_y_coordinates(points_picked["right_point"]),
        ]

        self.crown_top = {
            "pcd_left_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                points[0]
            ])),
            "pcd_right_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                points[1]
            ])),
            "pcd_center_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                (np.asarray(points[0]) + np.asarray(points[1])) / 2
            ])),
        }
        self.update_crown_top_point()

        self.crown_front = {
            "pcd_surface_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                swap_z_and_y_coordinates(points_picked["front_surface_point"]),
            ])),
        }

        self.update_crown_front_point()

    def get_points_on_the_tooth_surface_through_open3d(self):
        if os.path.isfile(f'{self.case_folder}{self.name}_picked_points.json'):
            points_picked = get_json_object(f'{self.case_folder}{self.name}_picked_points.json')
        elif Constants.ASK_USER_TO_PICK_POINTS:
            points = np.asarray(self.tooth.vertices)

            vis = o3d.visualization.VisualizerWithVertexSelection()
            vis.create_window()
            # vis.add_geometry(point_cloud)
            vis.add_geometry(self.tooth)
            vis.run()
            vis.destroy_window()

            picked_points = vis.get_picked_points()

            if len(picked_points) != 3:
                return

            left_point = picked_points[0].coord
            right_point = picked_points[1].coord
            front_surface_point = picked_points[2].coord

            points_picked = {
                "left_point": left_point,
                "right_point": right_point,
                "front_surface_point": front_surface_point,
            }

            save_as_json(points_picked, f'{self.case_folder}{self.name}_picked_points.json')
        else:
            return

        points = np.asarray([
            points_picked["left_point"], points_picked["right_point"], points_picked["front_surface_point"]
        ])

        points = points[points[:, 2].argsort()]
        if (self.tooth_id - 1) in Constants.MAXILLARY_RANGE:
            front_surface_point = points[-1]
            points = points[:-1]
        else:
            front_surface_point = points[0]
            points = points[1:]

        points = points[points[:, 1].argsort()]

        self.crown_top = {
            "pcd_left_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                points[0]
            ])),
            "pcd_right_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                points[1]
            ])),
            "pcd_center_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                (np.asarray(points[0]) + np.asarray(points[1])) / 2
            ])),
        }

        self.update_crown_top_point()

        self.crown_front = {
            "pcd_surface_point": o3d.geometry.PointCloud(o3d.utility.Vector3dVector([
                front_surface_point
            ])),
        }

        self.update_crown_front_point()

    # initialization of a tooth completes at this line

    # checked
    def get_surface_point(self, cur_point, slope):
        movement_distance = 1
        movement_threshold = 0.05
        while movement_distance > movement_threshold:
            next_point = cur_point + (slope * movement_distance)
            is_point_outside = self.is_point_outside_the_tooth([next_point])
            if is_point_outside:
                movement_distance /= 2
                next_point = cur_point

            cur_point = next_point
        return cur_point

    # section

    # checked
    def update_crown_top_point(self):
        self.crown_top["center_point"] = np.asarray(self.crown_top["pcd_center_point"].points)[0]
        self.crown_top["left_point"] = np.asarray(self.crown_top["pcd_left_point"].points)[0]
        self.crown_top["right_point"] = np.asarray(self.crown_top["pcd_right_point"].points)[0]

        tooth_center = self.crown["center_point"]

        self.crown_top["crown_slope"] = get_slope([self.crown_top["left_point"], self.crown_top["right_point"]])
        self.crown_top["center_slope"] = get_slope([tooth_center, self.crown_top["center_point"]])

    # checked
    def update_crown_front_point(self):
        self.crown_front["surface_point"] = np.asarray(self.crown_front["pcd_surface_point"].points)[0]
        self.crown_front["surface_point_slope"] = get_slope(
            [self.crown_top["center_point"], self.crown_front["surface_point"]]
        )

    def reorder_circle_points(self, start_point_idx):
        reordered_circle_points_list = []
        n_circle_points = len(self.circle_points['list'])
        idx = start_point_idx
        for i in range(n_circle_points):
            reordered_circle_points_list.append(self.circle_points['list'][idx])
            idx = (idx + 1) % n_circle_points

        self.circle_points['pcd'] = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(reordered_circle_points_list))

        self.update_snapping_points()

    def update_snapping_points(self):
        new_snapping_points = []
        for point in np.asarray(self.snapping_points["pcd"].points):
            new_snapping_points.append(list(point))

        self.snapping_points["list"] = new_snapping_points

    # checked
    def update_crown_center(self):
        self.crown["center_point"] = np.asarray(self.crown["pcd_center_point"].points)[0]

    def update_axes(self):
        pcd_axes: o3d.geometry.PointCloud = self.pcd_tooth_axes
        points = pcd_axes.points

        self.top_axis.update_axis(first_point=points[0], second_point=points[1])
        self.front_axis.update_axis(first_point=points[0], second_point=points[2])
        self.side_axis.update_axis(first_point=points[0], second_point=points[3])
        self.bone_center = np.asarray(points[4])

    # section: tooth movement functions

    # checked
    def rotate_using_euler_angles(
            self,
            x_rot_angle=0, y_rot_angle=0, z_rot_angle=0,
            rotating_center=None,
            move_back=False
    ):
        if self.is_dummy:
            return

        cur_tooth_center = copy.deepcopy(self.crown["center_point"])
        if rotating_center is None:
            rotating_center = cur_tooth_center

        for axis, rotation_angle in [('x', x_rot_angle), ('y', y_rot_angle), ('z', z_rot_angle)]:
            if rotation_angle == 0:
                continue

            r = Rotation.from_euler(axis, rotation_angle, degrees=True)
            rotation_matrix = r.as_matrix()

            self.tooth.rotate(rotation_matrix, rotating_center)
            self.crown["pcd_center_point"].rotate(rotation_matrix, rotating_center)
            self.update_crown_center()

            self.decimated_point_cloud.rotate(rotation_matrix, rotating_center)

            if self.triangle is not None:
                self.triangle.rotate(rotation_matrix, rotating_center)

            if self.crown_top is not None:
                self.crown_top["pcd_center_point"].rotate(rotation_matrix, rotating_center)
                self.crown_top["pcd_left_point"].rotate(rotation_matrix, rotating_center)
                self.crown_top["pcd_right_point"].rotate(rotation_matrix, rotating_center)
                self.update_crown_top_point()

                if self.pcd_tooth_axes is not None:
                    self.pcd_tooth_axes.rotate(rotation_matrix, rotating_center)
                    self.update_axes()

            if self.crown_front is not None:
                self.crown_front["pcd_surface_point"].rotate(rotation_matrix, rotating_center)
                self.update_crown_front_point()

            if self.snapping_points is not None:
                self.snapping_points["pcd"].rotate(rotation_matrix, rotating_center)
                self.update_snapping_points()

        if move_back:
            movement = cur_tooth_center - self.crown["center_point"]

            self.translate_coordinates(
                x_dist=movement[0],
                y_dist=movement[1],
                z_dist=movement[2],
            )

    # checked
    def rotate_using_quaternion(
            self,
            axis,
            angle,
            rotating_center=None,
            move_back=False
    ):
        if self.is_dummy:
            return

        cur_tooth_center = copy.deepcopy(self.crown["center_point"])
        if rotating_center is None:
            rotating_center = cur_tooth_center

        v = axis * np.sin(angle / 2)
        s = np.cos(angle / 2)

        q = np.array((s, v[0], v[1], v[2]))
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(q)

        self.tooth.rotate(rotation_matrix, rotating_center)
        self.crown["pcd_center_point"].rotate(rotation_matrix, rotating_center)
        self.update_crown_center()

        self.decimated_point_cloud.rotate(rotation_matrix, rotating_center)

        if self.triangle is not None:
            self.triangle.rotate(rotation_matrix, rotating_center)

        if self.crown_top is not None:
            self.crown_top["pcd_center_point"].rotate(rotation_matrix, rotating_center)
            self.crown_top["pcd_left_point"].rotate(rotation_matrix, rotating_center)
            self.crown_top["pcd_right_point"].rotate(rotation_matrix, rotating_center)
            self.update_crown_top_point()

        if self.pcd_tooth_axes is not None:
            self.pcd_tooth_axes.rotate(rotation_matrix, rotating_center)
            self.update_axes()

        if self.crown_front is not None:
            self.crown_front["pcd_surface_point"].rotate(rotation_matrix, rotating_center)
            self.update_crown_front_point()

        if self.snapping_points is not None:
            self.snapping_points["pcd"].rotate(rotation_matrix, rotating_center)
            self.update_snapping_points()

        if move_back:
            movement = cur_tooth_center - self.crown["center_point"]

            self.translate_coordinates(
                x_dist=movement[0],
                y_dist=movement[1],
                z_dist=movement[2],
            )

    # checked
    def translate_coordinates(
            self,
            x_dist: int = 0,
            y_dist: int = 0,
            z_dist: int = 0,
    ):
        if self.is_dummy:
            return

        self.tooth = self.tooth.translate([x_dist, y_dist, z_dist])
        self.crown["pcd_center_point"] = self.crown["pcd_center_point"].translate([x_dist, y_dist, z_dist])
        self.update_crown_center()

        self.decimated_point_cloud = self.decimated_point_cloud.translate([x_dist, y_dist, z_dist])

        if self.triangle is not None:
            self.triangle = self.triangle.translate([x_dist, y_dist, z_dist])

        if self.crown_top is not None:
            self.crown_top["pcd_center_point"] = self.crown_top["pcd_center_point"].translate([x_dist, y_dist, z_dist])
            self.crown_top["pcd_left_point"] = self.crown_top["pcd_left_point"].translate([x_dist, y_dist, z_dist])
            self.crown_top["pcd_right_point"] = self.crown_top["pcd_right_point"].translate([x_dist, y_dist, z_dist])
            self.update_crown_top_point()

        if self.pcd_tooth_axes is not None:
            self.pcd_tooth_axes = self.pcd_tooth_axes.translate([x_dist, y_dist, z_dist])
            self.update_axes()

        if self.crown_front is not None:
            self.crown_front["pcd_surface_point"] = self.crown_front["pcd_surface_point"].translate(
                [x_dist, y_dist, z_dist])
            self.update_crown_front_point()

        if self.snapping_points is not None:
            self.snapping_points["pcd"] = self.snapping_points["pcd"].translate([x_dist, y_dist, z_dist])
            self.update_snapping_points()

    # section: miscellaneous functions using movement functions and other open3d internal functions

    def translate_teeth_along_inter_teeth_axis(
            self,
            sphere,
            to_move: int = 1,
    ):
        # @param:
        #   to_move:
        #   1: move self tooth
        #   2: move both teeth
        #   3: move sphere tooth

        if self.is_dummy or sphere.is_dummy:
            return

        center_1 = self.crown["center_point"]
        center_2 = sphere.crown["center_point"]

        line12 = get_slope([np.asarray(center_1), np.asarray(center_2)])  # c1 -> c2
        cur_teeth_distance = self.check_collision(sphere)

        if to_move == 2:
            cur_teeth_distance /= 2

        movement_distance = (line12 * cur_teeth_distance)
        if to_move < 3:
            self.translate_coordinates(
                x_dist=movement_distance[0],
                y_dist=movement_distance[1],
                z_dist=movement_distance[2],
            )

        if to_move > 1:
            movement_distance = movement_distance * -1
            sphere.translate_coordinates(
                x_dist=movement_distance[0],
                y_dist=movement_distance[1],
                z_dist=movement_distance[2],
            )

    # To detect the collision between teeth using Raycasting operation in open3d
    def check_collision(
            self,
            sphere,
    ):
        scene = o3d.t.geometry.RaycastingScene()
        mesh = copy.deepcopy(self.tooth)
        mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

        _ = scene.add_triangles(mesh)

        query_points = o3d.core.Tensor(np.asarray(sphere.tooth.vertices), dtype=o3d.core.Dtype.Float32)

        signed_distance = scene.compute_signed_distance(query_points)
        minimum_distance = min(signed_distance.numpy())

        return minimum_distance

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

    def closest_point_in_point_cloud(
            self,
            sphere,
    ):
        scene = o3d.t.geometry.RaycastingScene()
        mesh = copy.deepcopy(self.tooth)
        mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

        _ = scene.add_triangles(mesh)

        query_points = o3d.core.Tensor(np.asarray(sphere.decimated_point_cloud.points), dtype=o3d.core.Dtype.Float32)

        signed_distance = scene.compute_signed_distance(query_points).numpy()
        closest_point_idx = np.argmin(signed_distance)

        return query_points[closest_point_idx], signed_distance[closest_point_idx]

    # To detect if a point is outside the tooth
    def is_point_outside_the_tooth(
            self,
            points,
    ):
        scene = o3d.t.geometry.RaycastingScene()
        mesh = copy.deepcopy(self.tooth)
        mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

        _ = scene.add_triangles(mesh)

        query_points = o3d.core.Tensor(np.asarray(points), dtype=o3d.core.Dtype.Float32)

        # Compute the signed distance for N random points
        signed_distance = scene.compute_signed_distance(query_points)
        minimum_distance = min(signed_distance.numpy())

        return minimum_distance > 0

        # query_point = o3d.core.Tensor(points, dtype=o3d.core.Dtype.Float32)
        # rays = np.concatenate(
        #     [query_point.numpy(),
        #      np.ones(query_point.shape, dtype=np.float32)],
        #     axis=-1
        # )

        # intersection_counts = scene.count_intersections(rays).numpy()
        # # A point is inside if the number of intersections with the scene is even
        # # This sssumes that inside and outside is we ll defined for the scene.
        # is_inside = intersection_counts % 2 == 1
        #
        # return not is_inside

    # To detect the collision between teeth
    def get_colliding_point_coordinates(
            self,
            sphere,
    ):
        tooth_1_points = np.asarray(self.decimated_point_cloud.points)
        tooth_2_points = np.asarray(sphere.decimated_point_cloud.points)

        closest_points = self.np_get_colliding_points(tooth_1_points, tooth_2_points)
        colliding_points = [tooth_1_points[closest_points[0][1]], tooth_2_points[closest_points[0][2]]]

        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(colliding_points))

        return pcd.get_center()

    # To get the colliding points between 2 teeth
    @staticmethod
    def get_colliding_points(
            tooth_1_points,
            tooth_2_points,
            n_points: int = 10
    ):
        closest_points_heap = []

        for i, tooth_1_point in enumerate(tooth_1_points):
            for j, tooth_2_point in enumerate(tooth_2_points):
                dist = get_euclidean_distance(tooth_1_point, tooth_2_point)
                heapq.heappush(
                    closest_points_heap,
                    (dist, i, j)
                )  # push into heap: distance between points ith of T1 and jth of T2

        closest_points = heapq.nsmallest(n_points, closest_points_heap)
        return closest_points

    @staticmethod
    def np_get_colliding_points(
            tooth_1_points,
            tooth_2_points
    ):
        num_points = tooth_2_points.shape[0]
        tooth_1_points_f = np.tile(tooth_1_points, num_points).reshape(num_points * num_points, 3)
        tooth_2_points_f = np.concatenate([tooth_2_points] * num_points).reshape(num_points * num_points, 3)

        tooth_1_points_i = np.repeat(np.arange(num_points), num_points)
        tooth_2_points_j = np.concatenate([np.arange(num_points)] * num_points)

        dist = np.sqrt(np.sum(np.square(tooth_2_points_f - tooth_1_points_f), axis=1))

        min_dist_idx = np.argmin(dist)
        min_dist = dist[min_dist_idx]
        min_i = tooth_1_points_i[min_dist_idx]
        min_j = tooth_2_points_j[min_dist_idx]
        return [(min_dist, min_i, min_j)]

    def match_lines_about_a_center_point(
            self,
            center_point: np.ndarray,
            initial_point: np.ndarray,
            final_point: np.ndarray = None,
            current_slope: np.ndarray = None,
            avg_slope: np.ndarray = None,
            move_back: bool = False
    ) -> tuple:
        if self.is_dummy:
            return 0, np.asarray([0, 0, 0])

        if current_slope is None:
            current_slope = get_slope([center_point, initial_point])

        if avg_slope is None:
            avg_slope = get_slope([center_point, final_point])

        distance = get_euclidean_distance(center_point, initial_point)
        avg_slope_point = center_point + (avg_slope * distance)

        if Constants.SHOW_GEOMETRIES:
            point1 = center_point - (initial_point * 7)
            point2 = initial_point + (current_slope * 7)
            line1 = get_line_segment_between_points(point1, point2, color=[1, 0, 0])

            point3 = center_point - (avg_slope * 7)
            point4 = avg_slope_point + (avg_slope * 7)
            line2 = get_line_segment_between_points(point3, point4, color=[0, 1, 0])

            o3d.visualization.draw_geometries([
                self.tooth,
                line1,
                line2,
            ])

        plane = get_plane_equation_from_points(
            points=[
                center_point,
                initial_point,
                avg_slope_point,
            ]
        )

        plane_normal = get_plane_normal(plane)

        rotation_angle = vector_angle_about_normal_axis(
            current_slope,
            avg_slope,
            plane_normal
        )

        self.rotate_using_quaternion(
            plane_normal,
            rotation_angle,
            rotating_center=center_point,
            move_back=move_back
        )

        return rotation_angle, plane_normal
