import open3d as o3d
import numpy as np
import os
import logging
import constants




# Open3d - Remove extra mesh


def extramesh_remove(input_file, cutter_file, output_merge_intersection, output_merge_not_intersection):
    try:
        logging.info('MESH REMOVE - START')
        teeth_merged_stl = os.path.join(
            constants.LOCAL_FOLDER, input_file)
        logging.info(teeth_merged_stl)
        merged_stl = o3d.io.read_triangle_mesh(teeth_merged_stl)
        logging.info(merged_stl)
        merged_stl.compute_vertex_normals()
        bounding_box_cutter_file = os.path.join(
            constants.LOCAL_FOLDER, cutter_file)
        cutter_stl = o3d.io.read_triangle_mesh(f'{bounding_box_cutter_file}')
        points = np.asarray(cutter_stl.vertices)
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(points))
        intersect_stl = merged_stl.crop(bounding_box)
        o3d.io.write_triangle_mesh(
            filename=output_merge_intersection,
            mesh=intersect_stl
        )
        merged_vertices = np.asarray(merged_stl.vertices)
        intersect_vertices = np.asarray(intersect_stl.vertices)
        merged_triangles = np.asarray(merged_stl.triangles)
        intersect_triangles = np.asarray(intersect_stl.triangles)
        merged_triangles_flattened = merged_triangles.flatten()
        merged_triangle_points = merged_vertices[merged_triangles_flattened]
        merged_triangle_points = np.reshape(
            merged_triangle_points, [merged_triangles.shape[0], 3, 3])
        intersect_triangles_flattened = intersect_triangles.flatten()
        intersect_triangle_points = intersect_vertices[intersect_triangles_flattened]
        intersect_triangle_points = np.reshape(intersect_triangle_points, [
                                               intersect_triangles.shape[0], 3, 3])
        triangle_mask = np.zeros(merged_triangle_points.shape[0])
        for vertex in intersect_triangle_points:
            diff = merged_triangle_points - vertex
            diff = np.sum(np.sum(diff, axis=1), axis=1)
            if np.any(diff == 0):
                triangle_mask[np.where(np.isin(diff, 0))] = 1
        triangle_mask = triangle_mask == 1
        merged_stl.remove_triangles_by_mask(list(triangle_mask))
        o3d.io.write_triangle_mesh(
            filename=output_merge_not_intersection,
            mesh=merged_stl
        )
        logging.info('MESH REMOVE - END')
    except Exception as e:
        logging.error("error" + e)

# Open3d - Remove extra mesh V1


def extramesh_remove_v1(mesh, filename):
    try:
        logging.info('MESH REMOVE V1 - START')
        vert = mesh.vertices
        zero = []
        for i in range(0, len(mesh.vertices)):
            if vert[i][0] == 0.0:
                zero.append(i)
        mesh.remove_vertices_by_index(zero)
        mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(mesh)
        o3d.io.write_triangle_mesh(
            filename=filename,
            mesh=mesh
        )
        logging.info('MESH REMOVE V1 - END')
    except Exception as e:
        logging.error("error" + e)