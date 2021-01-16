import os
import numpy as np
import open3d as o3d
from scipy import linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from skimage import measure


def read_obj(path):
    """
    Read .obj file and transform to numpy array.

    Args:
        path: path to the .obj file;
    
    Return:
        points: the point cloud array;
    """

    with open(path, "r") as file:
        lines = file.readlines()
    
    points = []
    for line in lines:
        coord = line.split()[1:]
        coord = [eval(x) for x in coord]
        points.append(coord)

    points = np.array(points)
    return points

def vecNorm(A, B):

    M = A.shape[0]
    N = B.shape[0]

    norm = np.zeros((M, N))

    A2 = np.sum(A ** 2, axis=1, keepdims=True)
    B2 = np.sum(B ** 2, axis=1, keepdims=True)
    AB = A @ B.T

    norm = A2 - 2*AB + B2.T

    return norm

def RBF_Reconstruction(pcl, c=1e-5, kc=1.0):
    """
    3D Reconstruction with RBF functions.

    Args:
        pcl: a point cloud;
        c: the distance parameter;
        kc: the kernel parameter;
    
    Returns:
        w: the weights vector;
    """

    ## retrieve points and normals from point cloud
    points = np.asarray(pcl.points)
    normals= np.asarray(pcl.normals)

    ## initialization
    N = points.shape[0]
    # kernels = [kernel(points[i]) for i in range(N)]

    A = np.zeros((3*N+1, N))
    b = np.zeros(3*N+1)

    ## surface points
    print("Adding surface points ...")
    # for i in range(N):
    #     k = kernels[i]
    #     for j in range(N):
    #         A[i, j] = k(points[j])

    A[:N, :N] = np.exp(-kc * vecNorm(points, points))
    
    ## off-surface points
    print("Adding off-surface points ...")
    # for i in range(N):
    #     k = kernels[i]
    #     for j in range(N):
    #         normal = normals[j]
    #         x = points[j] + c * normal

    #         A[i+N, j] = k(x)
    #         b[i+N] = c
    
    A[N:2*N, :N] = np.exp(-kc * vecNorm(points + c*normals, points))
    b[N:2*N] = c
    
    # for i in range(N):
    #     k = kernels[i]
    #     for j in range(N):
    #         normal = normals[j]
    #         x = points[j] - c * normal

    #         A[i+2*N, j] = k(x)
    #         b[i+2*N] = -c
    
    A[2*N:3*N, :N] = np.exp(-kc * vecNorm(points - c*normals, points))
    b[2*N:3*N] = -c
    
    ## constraints on weights
    A[-1,:] = 1
    
    ## solve the equation
    print("Solving the equation ...")
    w = linalg.lstsq(A, b)[0]

    return w


if __name__ == "__main__":
    root = os.getcwd()
    points = read_obj(os.path.join(root, "point_cloud_models", "kitten_04.obj"))
    points = points - points.mean(axis=0, keepdims=True)

    pcl = o3d.geometry.PointCloud()
    pcl.points = o3d.utility.Vector3dVector(points)
    ## downsample the point cloud 
    # pcl = pcl.voxel_down_sample(voxel_size=0.1)
    pcl.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))

    ## Arma
    kc = 5e4
    c  = 1e-2

    # ## dragon
    kc = 3e4
    c = 1e-2

    ## kitten
    kc = 12
    c  = 1e-3

    print(f"Reconstructing surface with parameter: kc = {kc}, c = {c}")
    w = RBF_Reconstruction(pcl, c, kc)

    ## distance field
    points = np.asarray(pcl.points)
    normals= np.asarray(pcl.normals)

    print("Creating grid ...")
    grid_range = np.linspace(points.min() - 1, points.max() + 1, 101)
    X, Y, Z = np.meshgrid(grid_range, grid_range, grid_range)
    grid = np.array([X, Y, Z])
    grid = grid.transpose((1, 2, 3, 0))
    grid = grid.reshape((-1, 3))

    print("Creating volume ...")
    dist = vecNorm(grid, points)
    dist = np.exp(-kc*dist)

    volume = dist @ w
    volume = volume.reshape((101, 101, 101))
    assert volume.min() < 0 <volume.max(), "Cannot find 0-level, please try another parameter settings."
    
    ## marching cube
    spacing = (points.max() - points.min() + 2) / 100
    verts, faces, normals, values = measure.marching_cubes(volume, 0.0, spacing=(spacing, spacing, spacing))

    ## reconstruct a point cloud
    pcl = o3d.geometry.PointCloud()
    pcl.points = o3d.utility.Vector3dVector(verts)

    ## create a mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
    o3d.visualization.draw_geometries([pcl, mesh], mesh_show_wireframe=True, mesh_show_back_face=True)