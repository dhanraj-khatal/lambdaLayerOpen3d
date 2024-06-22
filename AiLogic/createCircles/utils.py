import json

import numpy as np

import open3d as o3d

import vtk
import vedo

import sys
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
import math


def only_nearest(
        numNearest: int, pointCloud: o3d.geometry.PointCloud, labels: np.array
):
    changed = 0
    pcd_tree = o3d.geometry.KDTreeFlann(pointCloud)

    for i in range(labels.shape[0]):

        _, idx, _ = pcd_tree.search_knn_vector_3d(pointCloud.points[i], numNearest)

        count_teeth_labels = np.sum(labels[idx])

        if labels[i] == 0 and count_teeth_labels >= int(numNearest * 0.8):
            labels[i] = 1
            changed += 1
        elif labels[i] == 1 and count_teeth_labels <= int(numNearest * 0.2):
            labels[i] = 0
            changed += 1

    print("changed no of points: " + str(changed))

    return labels


def only_nearest_majority(
        numNearest: int, pointCloud: o3d.geometry.PointCloud, labels: np.array
):
    changed = 0
    pcd_tree = o3d.geometry.KDTreeFlann(pointCloud)

    for i in range(labels.shape[0]):

        _, idx, _ = pcd_tree.search_knn_vector_3d(pointCloud.points[i], numNearest)

        uni, counts = np.unique(labels[idx], return_counts=True)
        max_count_label = uni[np.argmax(counts)]

        if labels[i] != max_count_label and np.max(counts) > 0.8 * numNearest:
            labels[i] = max_count_label
            changed += 1

    print("changed no of points: " + str(changed))

    return labels


def obj_to_stl(obj_path: str, stl_path: str):
    mesh = vedo.io.load(obj_path)
    vedo.io.write(mesh, stl_path)


class MeshConv(object):
    def __init__(self, filename=None, warning=True):
        # initialize
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
            if self.filename[-3:].lower() == "vtp":
                self.read_vtp(self.filename)
            else:
                if self.warning:
                    print("Not support file type")

    def get_mesh_data_from_vtkPolyData(self):
        data = self.vtkPolyData

        n_triangles = data.GetNumberOfCells()
        n_points = data.GetNumberOfPoints()
        mesh_triangles = np.zeros([n_triangles, 9], dtype="float32")
        mesh_triangle_ids = np.zeros([n_triangles, 3], dtype="int32")
        mesh_points = np.zeros([n_points, 3], dtype="float32")

        for i in range(n_triangles):
            (
                mesh_triangles[i][0],
                mesh_triangles[i][1],
                mesh_triangles[i][2],
            ) = data.GetPoint(data.GetCell(i).GetPointId(0))
            (
                mesh_triangles[i][3],
                mesh_triangles[i][4],
                mesh_triangles[i][5],
            ) = data.GetPoint(data.GetCell(i).GetPointId(1))
            (
                mesh_triangles[i][6],
                mesh_triangles[i][7],
                mesh_triangles[i][8],
            ) = data.GetPoint(data.GetCell(i).GetPointId(2))
            mesh_triangle_ids[i][0] = data.GetCell(i).GetPointId(0)
            mesh_triangle_ids[i][1] = data.GetCell(i).GetPointId(1)
            mesh_triangle_ids[i][2] = data.GetCell(i).GetPointId(2)

        for i in range(n_points):
            mesh_points[i][0], mesh_points[i][1], mesh_points[i][2] = data.GetPoint(i)

        self.cells = mesh_triangles
        self.cell_ids = mesh_triangle_ids
        self.points = mesh_points

        # read cell arrays
        for i_attribute in range(self.vtkPolyData.GetCellData().GetNumberOfArrays()):
            print(
                "component dim: ",
                self.vtkPolyData.GetCellData()
                .GetArray(i_attribute)
                .GetNumberOfComponents(),
            )
            self.load_cell_attributes(
                self.vtkPolyData.GetCellData().GetArrayName(i_attribute),
                self.vtkPolyData.GetCellData()
                .GetArray(i_attribute)
                .GetNumberOfComponents(),
            )

    def read_vtp(self, vtp_filename):
        """
        update
            self.filename
            self.reader
            self.vtkPolyData
            self.cells
            self.cell_ids
            self.points
            self.cell_attributes
            self.point_attributes
        """
        #        self.filename = vtp_filename
        reader = vtk.vtkXMLPolyDataReader()
        reader.SetFileName(vtp_filename)
        reader.Update()
        self.reader = reader

        data = reader.GetOutput()
        self.vtkPolyData = data
        self.get_mesh_data_from_vtkPolyData()

    def load_cell_attributes(self, attribute_name, dim):
        self.cell_attributes[attribute_name] = np.zeros([self.cells.shape[0], dim])
        try:
            if dim == 1:
                for i in range(self.cells.shape[0]):
                    self.cell_attributes[attribute_name][i, 0] = (
                        self.vtkPolyData.GetCellData()
                        .GetArray(attribute_name)
                        .GetValue(i)
                    )
            elif dim == 2:
                for i in range(self.cells.shape[0]):
                    self.cell_attributes[attribute_name][i, 0] = (
                        self.vtkPolyData.GetCellData()
                        .GetArray(attribute_name)
                        .GetComponent(i, 0)
                    )
                    self.cell_attributes[attribute_name][i, 1] = (
                        self.vtkPolyData.GetCellData()
                        .GetArray(attribute_name)
                        .GetComponent(i, 1)
                    )
            elif dim == 3:
                for i in range(self.cells.shape[0]):
                    self.cell_attributes[attribute_name][i, 0] = (
                        self.vtkPolyData.GetCellData()
                        .GetArray(attribute_name)
                        .GetComponent(i, 0)
                    )
                    self.cell_attributes[attribute_name][i, 1] = (
                        self.vtkPolyData.GetCellData()
                        .GetArray(attribute_name)
                        .GetComponent(i, 1)
                    )
                    self.cell_attributes[attribute_name][i, 2] = (
                        self.vtkPolyData.GetCellData()
                        .GetArray(attribute_name)
                        .GetComponent(i, 2)
                    )
        except:
            if self.warning:
                print(
                    'No cell attribute named "{0}" in file: {1}'.format(
                        attribute_name, self.filename
                    )
                )

    def to_obj(self, obj_filename):
        with open(obj_filename, "w") as f:
            for i_point in self.points:
                f.write("v {} {} {}\n".format(i_point[0], i_point[1], i_point[2]))
            print(len(self.cell_ids), 'aaaaaaaaaaaaaaaaaa')
            for i_label_cell_id in self.cell_ids:
                i_cell = self.cell_ids[i_label_cell_id]
                f.write(
                    "f {}//{} {}//{} {}//{}\n".format(
                        i_cell[0] + 1,
                        i_cell[0] + 1,
                        i_cell[1] + 1,
                        i_cell[1] + 1,
                        i_cell[2] + 1,
                        i_cell[2] + 1,
                    )
                )
    
    def to_obj_label(self, obj_filename, label_num):
        with open(obj_filename, "w") as f:
            for i_point in self.points:
                f.write("v {} {} {}\n".format(i_point[0], i_point[1], i_point[2]))

            f.write("g mmGroup{}\n".format(int(label_num)))
            label_cell_ids = np.where(self.cell_attributes["Label"] == label_num)[0]
            for i_label_cell_id in label_cell_ids:
                i_cell = self.cell_ids[i_label_cell_id]
                f.write(
                    "f {}//{} {}//{} {}//{}\n".format(
                        i_cell[0] + 1,
                        i_cell[0] + 1,
                        i_cell[1] + 1,
                        i_cell[1] + 1,
                        i_cell[2] + 1,
                        i_cell[2] + 1,
                    )
                )


class MeshRefine(object):
    def __init__(self, filename=None, warning=False):
        # initialize
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
            if self.filename[-3:].lower() == 'vtp':
                self.read_vtp(self.filename)
            elif self.filename[-3:].lower() == 'stl':
                self.read_stl(self.filename)
            elif self.filename[-3:].lower() == 'obj':
                self.read_obj(self.filename)
            else:
                if self.warning:
                    print('Not support file type')

    def get_mesh_data_from_vtkPolyData(self):
        data = self.vtkPolyData

        n_triangles = data.GetNumberOfCells()
        n_points = data.GetNumberOfPoints()
        mesh_triangles = np.zeros([n_triangles, 9], dtype='float32')
        mesh_triangle_ids = np.zeros([n_triangles, 3], dtype='int32')
        mesh_points = np.zeros([n_points, 3], dtype='float32')

        for i in range(n_triangles):
            mesh_triangles[i][0], mesh_triangles[i][1], mesh_triangles[i][2] = data.GetPoint(
                data.GetCell(i).GetPointId(0))
            mesh_triangles[i][3], mesh_triangles[i][4], mesh_triangles[i][5] = data.GetPoint(
                data.GetCell(i).GetPointId(1))
            mesh_triangles[i][6], mesh_triangles[i][7], mesh_triangles[i][8] = data.GetPoint(
                data.GetCell(i).GetPointId(2))
            mesh_triangle_ids[i][0] = data.GetCell(i).GetPointId(0)
            mesh_triangle_ids[i][1] = data.GetCell(i).GetPointId(1)
            mesh_triangle_ids[i][2] = data.GetCell(i).GetPointId(2)

        for i in range(n_points):
            mesh_points[i][0], mesh_points[i][1], mesh_points[i][2] = data.GetPoint(i)

        self.cells = mesh_triangles
        self.cell_ids = mesh_triangle_ids
        self.points = mesh_points

        # read point arrays
        for i_attribute in range(self.vtkPolyData.GetPointData().GetNumberOfArrays()):
            #            print(self.vtkPolyData.GetPointData().GetArrayName(i_attribute))
            #            print(self.vtkPolyData.GetPointData().GetArray(i_attribute).GetNumberOfComponents())
            self.load_point_attributes(self.vtkPolyData.GetPointData().GetArrayName(i_attribute),
                                       self.vtkPolyData.GetPointData().GetArray(i_attribute).GetNumberOfComponents())

        # read cell arrays
        for i_attribute in range(self.vtkPolyData.GetCellData().GetNumberOfArrays()):
            #            print(self.vtkPolyData.GetCellData().GetArrayName(i_attribute))
            #            print(self.vtkPolyData.GetCellData().GetArray(i_attribute).GetNumberOfComponents())
            self.load_cell_attributes(self.vtkPolyData.GetCellData().GetArrayName(i_attribute),
                                      self.vtkPolyData.GetCellData().GetArray(i_attribute).GetNumberOfComponents())

    def read_vtp(self, vtp_filename):
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
        #        self.filename = vtp_filename
        reader = vtk.vtkXMLPolyDataReader()
        reader.SetFileName(vtp_filename)
        reader.Update()
        self.reader = reader

        data = reader.GetOutput()
        self.vtkPolyData = data
        self.get_mesh_data_from_vtkPolyData()

    def load_point_attributes(self, attribute_name, dim):
        self.point_attributes[attribute_name] = np.zeros([self.points.shape[0], dim])
        try:
            if dim == 1:
                for i in range(self.points.shape[0]):
                    self.point_attributes[attribute_name][i, 0] = self.vtkPolyData.GetPointData().GetArray(
                        attribute_name).GetValue(i)
            elif dim == 2:
                for i in range(self.points.shape[0]):
                    self.point_attributes[attribute_name][i, 0] = self.vtkPolyData.GetPointData().GetArray(
                        attribute_name).GetComponent(i, 0)
                    self.point_attributes[attribute_name][i, 1] = self.vtkPolyData.GetPointData().GetArray(
                        attribute_name).GetComponent(i, 1)
            elif dim == 3:
                for i in range(self.points.shape[0]):
                    self.point_attributes[attribute_name][i, 0] = self.vtkPolyData.GetPointData().GetArray(
                        attribute_name).GetComponent(i, 0)
                    self.point_attributes[attribute_name][i, 1] = self.vtkPolyData.GetPointData().GetArray(
                        attribute_name).GetComponent(i, 1)
                    self.point_attributes[attribute_name][i, 2] = self.vtkPolyData.GetPointData().GetArray(
                        attribute_name).GetComponent(i, 2)
        except:
            if self.warning:
                print('No cell attribute named "{0}" in file: {1}'.format(attribute_name, self.filename))

    def get_point_curvatures(self, method='mean'):
        curv = vtk.vtkCurvatures()
        curv.SetInputData(self.vtkPolyData)
        if method == 'mean':
            curv.SetCurvatureTypeToMean()
        elif method == 'max':
            curv.SetCurvatureTypeToMaximum()
        elif method == 'min':
            curv.SetCurvatureTypeToMinimum()
        elif method == 'Gaussian':
            curv.SetCurvatureTypeToGaussian()
        else:
            curv.SetCurvatureTypeToMean()
        curv.Update()

        n_points = self.vtkPolyData.GetNumberOfPoints()
        self.point_attributes['Curvature'] = np.zeros([n_points, 1])
        for i in range(n_points):
            self.point_attributes['Curvature'][i] = curv.GetOutput().GetPointData().GetArray(0).GetValue(i)

    def get_cell_curvatures(self, method='mean'):
        self.get_point_curvatures(method=method)
        self.cell_attributes['Curvature'] = np.zeros([self.cells.shape[0], 1])

        # optimized way
        tmp_cell_curvts = self.point_attributes['Curvature'][self.cell_ids].squeeze()
        self.cell_attributes['Curvature'] = np.mean(tmp_cell_curvts, axis=-1).reshape([tmp_cell_curvts.shape[0], 1])

    def load_cell_attributes(self, attribute_name, dim):
        self.cell_attributes[attribute_name] = np.zeros([self.cells.shape[0], dim])
        try:
            if dim == 1:
                for i in range(self.cells.shape[0]):
                    self.cell_attributes[attribute_name][i, 0] = self.vtkPolyData.GetCellData().GetArray(
                        attribute_name).GetValue(i)
            elif dim == 2:
                for i in range(self.cells.shape[0]):
                    self.cell_attributes[attribute_name][i, 0] = self.vtkPolyData.GetCellData().GetArray(
                        attribute_name).GetComponent(i, 0)
                    self.cell_attributes[attribute_name][i, 1] = self.vtkPolyData.GetCellData().GetArray(
                        attribute_name).GetComponent(i, 1)
            elif dim == 3:
                for i in range(self.cells.shape[0]):
                    self.cell_attributes[attribute_name][i, 0] = self.vtkPolyData.GetCellData().GetArray(
                        attribute_name).GetComponent(i, 0)
                    self.cell_attributes[attribute_name][i, 1] = self.vtkPolyData.GetCellData().GetArray(
                        attribute_name).GetComponent(i, 1)
                    self.cell_attributes[attribute_name][i, 2] = self.vtkPolyData.GetCellData().GetArray(
                        attribute_name).GetComponent(i, 2)
        except:
            if self.warning:
                print('No cell attribute named "{0}" in file: {1}'.format(attribute_name, self.filename))

    def set_cell_labels(self, label_dict, tol=0.01):
        '''
        update:
            self.cell_attributes['Label']
        '''
        from scipy.spatial import distance_matrix
        self.cell_attributes['Label'] = np.zeros([self.cell_ids.shape[0], 1])

        cell_centers = (self.cells[:, 0:3] + self.cells[:, 3:6] + self.cells[:, 6:9]) / 3.0
        for i_label in label_dict:
            i_label_cell_centers = (label_dict[i_label][:, 0:3] + label_dict[i_label][:, 3:6] + label_dict[i_label][:,
                                                                                                6:9]) / 3.0
            D = distance_matrix(cell_centers, i_label_cell_centers)

            if len(np.argwhere(D <= tol)) > i_label_cell_centers.shape[0]:
                sys.exit('tolerance ({0}) is too large, please adjust.'.format(tol))
            elif len(np.argwhere(D <= tol)) < i_label_cell_centers.shape[0]:
                sys.exit('tolerance ({0}) is too small, please adjust.'.format(tol))
            else:
                for i in range(i_label_cell_centers.shape[0]):
                    label_id = np.argwhere(D <= tol)[i][0]
                    self.cell_attributes['Label'][label_id, 0] = int(i_label)

    def get_cell_edges(self):
        '''
        update:
            self.cell_attributes['Edge']
        '''
        self.cell_attributes['Edge'] = np.zeros([self.cell_ids.shape[0], 3])

        for i_count in range(self.cell_ids.shape[0]):
            v1 = self.points[self.cell_ids[i_count, 0], :] - self.points[self.cell_ids[i_count, 1], :]
            v2 = self.points[self.cell_ids[i_count, 1], :] - self.points[self.cell_ids[i_count, 2], :]
            v3 = self.points[self.cell_ids[i_count, 0], :] - self.points[self.cell_ids[i_count, 2], :]
            self.cell_attributes['Edge'][i_count, 0] = np.linalg.norm(v1)
            self.cell_attributes['Edge'][i_count, 1] = np.linalg.norm(v2)
            self.cell_attributes['Edge'][i_count, 2] = np.linalg.norm(v3)

    def get_cell_normals(self):
        data = self.vtkPolyData
        n_triangles = data.GetNumberOfCells()
        # normal
        v1 = np.zeros([n_triangles, 3], dtype='float32')
        v2 = np.zeros([n_triangles, 3], dtype='float32')
        v1[:, 0] = self.cells[:, 0] - self.cells[:, 3]
        v1[:, 1] = self.cells[:, 1] - self.cells[:, 4]
        v1[:, 2] = self.cells[:, 2] - self.cells[:, 5]
        v2[:, 0] = self.cells[:, 3] - self.cells[:, 6]
        v2[:, 1] = self.cells[:, 4] - self.cells[:, 7]
        v2[:, 2] = self.cells[:, 5] - self.cells[:, 8]
        mesh_normals = np.cross(v1, v2)
        mesh_normal_length = np.linalg.norm(mesh_normals, axis=1)
        mesh_normals[:, 0] /= mesh_normal_length[:]
        mesh_normals[:, 1] /= mesh_normal_length[:]
        mesh_normals[:, 2] /= mesh_normal_length[:]
        self.cell_attributes['Normal'] = mesh_normals

    def compute_cell_attributes_by_knn(self, given_cells, given_cell_attributes, attribute_name, k=3, refine=False):
        '''
        inputs:
            given_cells: [n, 9] numpy array
            given_cell_attributes: [n, 1] numpy array
        update:
            self.cell_attributes[attribute_name]
        '''
        from sklearn.neighbors import KNeighborsClassifier
        if given_cell_attributes.shape[1] == 1:
            self.cell_attributes[attribute_name] = np.zeros([self.cells.shape[0], 1])
            neigh = KNeighborsClassifier(n_neighbors=k)
            neigh.fit(given_cells, given_cell_attributes.ravel())
            self.cell_attributes[attribute_name][:, 0] = neigh.predict(self.cells)
            self.cell_attributes[attribute_name + '_proba'] = neigh.predict_proba(self.cells)

            if refine:
                self.graph_cut_refinement(self.cell_attributes[attribute_name + '_proba'])
        else:
            if self.warning:
                print('Only support 1D attribute')

    def graph_cut_refinement(self, patch_prob_output):
        from pygco import cut_from_graph
        round_factor = 100
        patch_prob_output[patch_prob_output < 1.0e-6] = 1.0e-6

        # unaries
        unaries = -round_factor * np.log10(patch_prob_output)
        unaries = unaries.astype(np.int32)
        unaries = unaries.reshape(-1, patch_prob_output.shape[1])

        # parawise
        pairwise = (1 - np.eye(patch_prob_output.shape[1], dtype=np.int32))

        # edges
        self.get_cell_normals()
        normals = self.cell_attributes['Normal'][:]
        cells = self.cells[:]
        cell_ids = self.cell_ids[:]
        barycenters = (cells[:, 0:3] + cells[:, 3:6] + cells[:, 6:9]) / 3.0

        lambda_c = 30
        edges = np.empty([1, 3], order='C')
        for i_node in range(cells.shape[0]):
            # Find neighbors
            nei = np.sum(np.isin(cell_ids, cell_ids[i_node, :]), axis=1)
            nei_id = np.where(nei == 2)
            for i_nei in nei_id[0][:]:
                if i_node < i_nei:
                    cos_theta = np.dot(normals[i_node, 0:3], normals[i_nei, 0:3]) / np.linalg.norm(
                        normals[i_node, 0:3]) / np.linalg.norm(normals[i_nei, 0:3])
                    if cos_theta >= 1.0:
                        cos_theta = 0.9999
                    theta = np.arccos(cos_theta)
                    phi = np.linalg.norm(barycenters[i_node, :] - barycenters[i_nei, :])
                    if theta > np.pi / 2.0:
                        edges = np.concatenate(
                            (edges, np.array([i_node, i_nei, -math.log10(theta / np.pi) * phi]).reshape(1, 3)), axis=0)
                    else:
                        beta = 1 + np.linalg.norm(np.dot(normals[i_node, 0:3], normals[i_nei, 0:3]))
                        edges = np.concatenate(
                            (edges, np.array([i_node, i_nei, -beta * math.log10(theta / np.pi) * phi]).reshape(1, 3)),
                            axis=0)
        edges = np.delete(edges, 0, 0)
        edges[:, 2] *= lambda_c * round_factor
        edges = edges.astype(np.int32)

        refine_labels = cut_from_graph(edges, unaries, pairwise)
        refine_labels = refine_labels.reshape([-1, 1])

        # output refined result
        self.cell_attributes['Label'] = refine_labels

    def update_cell_ids_and_points(self):
        '''
        call when self.cells is modified
        update
            self.cell_ids
            self.points
        '''
        rdt_points = self.cells.reshape([int(self.cells.shape[0] * 3), 3])
        self.points, idx = np.unique(rdt_points, return_inverse=True, axis=0)
        self.cell_ids = idx.reshape([-1, 3])

        if self.warning:
            print('Warning! self.cell_attributes are reset and need to be updated!')
        self.cell_attributes = dict()  # reset
        self.point_attributes = dict()  # reset
        self.update_vtkPolyData()

    def update_vtkPolyData(self):
        '''
        call this function when manipulating self.cells, self.cell_ids, or self.points
        '''
        vtkPolyData = vtk.vtkPolyData()
        points = vtk.vtkPoints()
        cells = vtk.vtkCellArray()

        points.SetData(numpy_to_vtk(self.points))
        cells.SetCells(len(self.cell_ids),
                       numpy_to_vtkIdTypeArray(np.hstack((np.ones(len(self.cell_ids))[:, None] * 3,
                                                          self.cell_ids)).astype(np.int64).ravel(),
                                               deep=1))
        vtkPolyData.SetPoints(points)
        vtkPolyData.SetPolys(cells)

        # update point_attributes
        for i_key in self.point_attributes.keys():
            point_attribute = vtk.vtkDoubleArray()
            point_attribute.SetName(i_key);
            if self.point_attributes[i_key].shape[1] == 1:
                point_attribute.SetNumberOfComponents(self.point_attributes[i_key].shape[1])
                for i_attribute in self.point_attributes[i_key]:
                    point_attribute.InsertNextTuple1(i_attribute)
                vtkPolyData.GetPointData().AddArray(point_attribute)
            #                vtkPolyData.GetPointData().SetScalars(cell_attribute)
            elif self.point_attributes[i_key].shape[1] == 2:
                point_attribute.SetNumberOfComponents(self.point_attributes[i_key].shape[1])
                for i_attribute in self.point_attributes[i_key]:
                    point_attribute.InsertNextTuple2(i_attribute[0], i_attribute[1])
                vtkPolyData.GetPointData().AddArray(point_attribute)
            #                vtkPolyData.GetPointData().SetVectors(cell_attribute)
            elif self.point_attributes[i_key].shape[1] == 3:
                point_attribute.SetNumberOfComponents(self.point_attributes[i_key].shape[1])
                for i_attribute in self.point_attributes[i_key]:
                    point_attribute.InsertNextTuple3(i_attribute[0], i_attribute[1], i_attribute[2])
                vtkPolyData.GetPointData().AddArray(point_attribute)
            #                vtkPolyData.GetPointData().SetVectors(cell_attribute)
            else:
                if self.warning:
                    print('Check attribute dimension, only support 1D, 2D, and 3D now')

        # update cell_attributes
        for i_key in self.cell_attributes.keys():
            cell_attribute = vtk.vtkDoubleArray()
            cell_attribute.SetName(i_key);
            if self.cell_attributes[i_key].shape[1] == 1:
                cell_attribute.SetNumberOfComponents(self.cell_attributes[i_key].shape[1])
                for i_attribute in self.cell_attributes[i_key]:
                    cell_attribute.InsertNextTuple1(i_attribute)
                vtkPolyData.GetCellData().AddArray(cell_attribute)
            #                vtkPolyData.GetCellData().SetScalars(cell_attribute)
            elif self.cell_attributes[i_key].shape[1] == 2:
                cell_attribute.SetNumberOfComponents(self.cell_attributes[i_key].shape[1])
                for i_attribute in self.cell_attributes[i_key]:
                    cell_attribute.InsertNextTuple2(i_attribute[0], i_attribute[1])
                vtkPolyData.GetCellData().AddArray(cell_attribute)
            #                vtkPolyData.GetCellData().SetVectors(cell_attribute)
            elif self.cell_attributes[i_key].shape[1] == 3:
                cell_attribute.SetNumberOfComponents(self.cell_attributes[i_key].shape[1])
                for i_attribute in self.cell_attributes[i_key]:
                    cell_attribute.InsertNextTuple3(i_attribute[0], i_attribute[1], i_attribute[2])
                vtkPolyData.GetCellData().AddArray(cell_attribute)
            #                vtkPolyData.GetCellData().SetVectors(cell_attribute)
            else:
                if self.warning:
                    print('Check attribute dimension, only support 1D, 2D, and 3D now')

        vtkPolyData.Modified()
        self.vtkPolyData = vtkPolyData

    def extract_largest_region(self):
        connect = vtk.vtkPolyDataConnectivityFilter()
        connect.SetInputData(self.vtkPolyData)
        connect.SetExtractionModeToLargestRegion()
        connect.Update()

        self.vtkPolyData = connect.GetOutput()
        self.get_mesh_data_from_vtkPolyData()
        if self.warning:
            print('Warning! self.cell_attributes are reset and need to be updated!')
        self.cell_attributes = dict()  # reset
        self.point_attributes = dict()  # reset

    def to_vtp(self, vtp_filename):
        self.update_vtkPolyData()

        if vtk.VTK_MAJOR_VERSION <= 5:
            self.vtkPolyData.Update()

        writer = vtk.vtkXMLPolyDataWriter();
        writer.SetFileName("{0}".format(vtp_filename));
        if vtk.VTK_MAJOR_VERSION <= 5:
            writer.SetInput(self.vtkPolyData)
        else:
            writer.SetInputData(self.vtkPolyData)
        writer.Write()


maxillary_label_map = {
    1: 1,
    2: 2,
    3: 3,
    4: 4,
    5: 5,
    6: 6,
    7: 7,
    8: 8,
    9: 9,
    10: 10,
    11: 11,
    12: 12,
    13: 13,
    14: 14,
    15: 15,
    16: 16
}

mandibular_label_map = {
    1: 17,
    2: 18,
    3: 19,
    4: 20,
    5: 21,
    6: 22,
    7: 23,
    8: 24,
    9: 25,
    10: 26,
    11: 27,
    12: 28,
    13: 29,
    14: 30,
    15: 31,
    16: 32,
}


def label_to_tooth_num(type: str, label: int) -> int:
    if type == "maxillary":
        return maxillary_label_map[label]
    else:
        return mandibular_label_map[label]


def numpy2vtk(arr, dtype=None, deep=True, name=""):
    """Convert a numpy array into a `vtkDataArray`. Use dtype='id' for vtkIdTypeArray objects."""
    # https://github.com/Kitware/VTK/blob/master/Wrapping/Python/vtkmodules/util/numpy_support.py
    if arr is None:
        return None

    arr = np.ascontiguousarray(arr)

    if dtype == 'id':
        varr = numpy_to_vtkIdTypeArray(arr.astype(np.int64), deep=deep)
    elif dtype:
        varr = numpy_to_vtk(arr.astype(dtype), deep=deep)
    else:
        # let numpy_to_vtk() decide what is best type based on arr type
        varr = numpy_to_vtk(arr, deep=deep)

    if name:
        varr.SetName(name)
    return varr


def geodesic(vmesh, start, end):
    """
    Dijkstra algorithm to compute the geodesic line.
    Takes as input a polygonal mesh and performs a single source shortest path calculation.

    Parameters
    ----------
    vmesh : vedo.Mesh

    start : int, list
        start vertex index or close point `[x,y,z]`

    end :  int, list
        end vertex index or close point `[x,y,z]`

    .. hint:: geodesic.py
        .. image:: https://vedo.embl.es/images/advanced/geodesic.png
    """
    if vedo.utils.isSequence(start):
        cc = vmesh.points()
        pa = vedo.Points(cc)
        start = pa.closestPoint(start, returnIds=True)
        end = pa.closestPoint(end, returnIds=True)

    dijkstra = vtk.vtkDijkstraGraphGeodesicPath()
    dijkstra.SetInputData(vmesh.polydata())
    dijkstra.SetStartVertex(end)  # inverted in vtk
    dijkstra.SetEndVertex(start)
    dijkstra.Update()

    weights = vtk.vtkDoubleArray()
    dijkstra.GetCumulativeWeights(weights)

    idlist = dijkstra.GetIdList()
    ids = [idlist.GetId(i) for i in range(idlist.GetNumberOfIds())]

    length = weights.GetMaxId() + 1
    arr = np.zeros(length)
    for i in range(length):
        arr[i] = weights.GetTuple(i)[0]

    poly = dijkstra.GetOutput()

    vdata = numpy2vtk(arr)
    vdata.SetName("CumulativeWeights")
    poly.GetPointData().AddArray(vdata)

    vdata2 = numpy2vtk(ids, dtype=np.uint)
    vdata2.SetName("VertexIDs")
    poly.GetPointData().AddArray(vdata2)
    poly.GetPointData().Modified()

    dmesh = vedo.Mesh(poly, c='k')
    prop = vtk.vtkProperty()
    prop.SetLineWidth(6)
    prop.SetOpacity(1)
    dmesh.SetProperty(prop)
    dmesh.property = prop
    dmesh.name = "GeodesicLine"

    return dmesh


class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """

    def default(self, obj):
        if isinstance(obj, (np.int_, np.intc, np.intp, np.int8,
                            np.int16, np.int32, np.int64, np.uint8,
                            np.uint16, np.uint32, np.uint64)):
            return int(obj)
        elif isinstance(obj, (np.float_, np.float16, np.float32,
                              np.float64)):
            return float(obj)
        elif isinstance(obj, (np.ndarray,)):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def save_as_json(data, file_name):
    out_file = open(file_name, "w")
    json.dump(data, out_file, indent=4, cls=NumpyEncoder)
    out_file.close()


def get_json_object(filepath: str):
    # Opening JSON file
    f = open(filepath)

    # returns JSON object as a dictionary
    json_obj = json.load(f)

    # Closing file
    f.close()

    return json_obj