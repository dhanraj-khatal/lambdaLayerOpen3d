import numpy as np
import open3d as o3d 
import sys
from json import JSONEncoder
import json
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)
class BoundMesh(object):
    def __init__(self, filename = None, warning=False):
        #initialize
        self.warning = warning
        self.reader = None
        self.vtkPolyData = None
        self.cells = np.array([])
        self.cell_ids = np.array([])
        self.points = np.array([])
        self.point_attributes = dict()
        self.cell_attributes = dict()
        self.filename = filename
        if self.filename != None:
            if self.filename[-3:].lower() == 'stl':
                self.read_stl(self.filename)


    def get_mesh_data_from_vtkPolyData(self):
        data = self.vtkPolyData

        n_triangles = data.GetNumberOfCells()
        n_points = data.GetNumberOfPoints()
        mesh_triangles = np.zeros([n_triangles, 9], dtype='float32')
        mesh_triangle_ids = np.zeros([n_triangles, 3], dtype='int32')
        mesh_points = np.zeros([n_points, 3], dtype='float32')

        for i in range(n_triangles):
            mesh_triangles[i][0], mesh_triangles[i][1], mesh_triangles[i][2] = data.GetPoint(data.GetCell(i).GetPointId(0))
            mesh_triangles[i][3], mesh_triangles[i][4], mesh_triangles[i][5] = data.GetPoint(data.GetCell(i).GetPointId(1))
            mesh_triangles[i][6], mesh_triangles[i][7], mesh_triangles[i][8] = data.GetPoint(data.GetCell(i).GetPointId(2))
            mesh_triangle_ids[i][0] = data.GetCell(i).GetPointId(0)
            mesh_triangle_ids[i][1] = data.GetCell(i).GetPointId(1)
            mesh_triangle_ids[i][2] = data.GetCell(i).GetPointId(2)

        for i in range(n_points):
            mesh_points[i][0], mesh_points[i][1], mesh_points[i][2] = data.GetPoint(i)

        self.cells = mesh_triangles
        self.cell_ids = mesh_triangle_ids
        self.points = mesh_points


    def read_stl(self, stl_filename):
        '''
        update
            self.filename
            self.reader
            self.vtkPolyData
            self.cells
            self.cell_ids
            self.points
            self.cell_attributes
            self.point_attributes
        '''
        reader = vtk.vtkSTLReader()
        reader.SetFileName(stl_filename)
        reader.Update()
        self.reader = reader

        data = reader.GetOutput()
        self.vtkPolyData = data
        self.get_mesh_data_from_vtkPolyData()


    def get_ordered_boundary_points(self):
        '''
        output: ordered boundary_points [n, 3] nparray
        '''
        featureEdges = vtk.vtkFeatureEdges()
        featureEdges.SetInputData(self.vtkPolyData)
        featureEdges.BoundaryEdgesOn()
        featureEdges.FeatureEdgesOff()
        featureEdges.ManifoldEdgesOff()
        featureEdges.NonManifoldEdgesOff()
        featureEdges.Update()

        clean_border_edges = vtk.vtkCleanPolyData()
        clean_border_edges.SetInputConnection(featureEdges.GetOutputPort())

        border_strips = vtk.vtkStripper()
        border_strips.SetJoinContiguousSegments(True)
        border_strips.SetInputConnection(clean_border_edges.GetOutputPort())
        border_strips.Update()

        border_polygons = vtk.vtkCellArray()
        border_polygons.SetNumberOfCells(border_strips.GetOutput().GetNumberOfCells())

        boundary_points_per_polygon = {}
        count = 1
        for cell in range(border_strips.GetOutput().GetNumberOfCells()):
            border_points = vtk.vtkPoints()
            points_in_border = 0

            polygon_n_boundary_points = border_strips.GetOutput().GetCell(cell).GetNumberOfPoints()
            if polygon_n_boundary_points < 40:
                continue

            polygon = vtk.vtkPolygon()
            polygon.GetPointIds().SetNumberOfIds(polygon_n_boundary_points)
            for point_in_cell in range(polygon_n_boundary_points):
                point_in_border = point_in_cell + points_in_border
                border_points.InsertNextPoint(
                    border_strips.GetOutput().GetCell(cell).GetPoints().GetPoint(point_in_cell))
                polygon.GetPointIds().SetId(point_in_cell, point_in_border)
            border_polygons.InsertNextCell(polygon)
            points_in_polygon = polygon.GetNumberOfPoints()
            points_in_border += points_in_polygon

            polygon_boundary_points = np.zeros([polygon_n_boundary_points, 3], dtype='float32')
            for i in range(polygon_n_boundary_points):
                polygon_boundary_points[i][0], polygon_boundary_points[i][1], polygon_boundary_points[i][
                    2] = border_points.GetPoint(i)

            boundary_points_per_polygon[count] = polygon_boundary_points

            count += 1

        max_length_polygon = 0
        max_length_polygon_id = 1
        for (polygon_id, boundary_points) in boundary_points_per_polygon.items():
            if len(boundary_points) > max_length_polygon:
                max_length_polygon = len(boundary_points)
                max_length_polygon_id = polygon_id


        return boundary_points_per_polygon[max_length_polygon_id]


# if __name__ == "__main__":
#     path_to_stl_file = "baseMesh_BJS_test_02.stl"  # pass the stl file path here
#     path_to_ply_output = "dummy_output.ply"
def process_vertex_reorder(path_to_stl_file,path_to_json_output):
    path_to_ply_output = "dummy_output.ply"
    only_teeth_mesh = BoundMesh(path_to_stl_file)
    bound_points = only_teeth_mesh.get_ordered_boundary_points()
    print(bound_points,"boud")
    with open(path_to_json_output, "w") as write_file:
        json.dump(bound_points, write_file, cls=NumpyArrayEncoder)
    # ply = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(bound_points))
    # o3d.visualization.draw_geometries([ply])

    # o3d.io.write_point_cloud(path_to_ply_output, ply)
