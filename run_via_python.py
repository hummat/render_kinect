
from ctypes import *
import time
import numpy as np
import os
import trimesh
import matplotlib.pyplot as plt
import open3d as o3d


lib = cdll.LoadLibrary("/home/wink_do/PycharmProjects/render_kinect/lib/libkinectSim.so")

lib.simulate.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32, ndim=2, flags='C_CONTIGUOUS'), c_int, np.ctypeslib.ndpointer(dtype=np.int32, ndim=2, flags='C_CONTIGUOUS'), c_int, np.ctypeslib.ndpointer(dtype=np.float32, ndim=2, flags='C_CONTIGUOUS')]
lib.simulate.restype = None

class KinectSim:

    @staticmethod
    def simulate(verts, faces, out_depth):
        lib.simulate(verts, len(verts), faces, len(faces), out_depth)
     

    
name = "144_scan_small_cup"
front = True
f_kinect_world = np.array([[0.18889654314248513, -0.5491412049704937, 0.8141013875353555, -1.29464394785277], [-0.9455020122657823, -0.325615105774899, -0.0002537971720279581, -0.009541282377521727], [0.2652233842565174, -0.7696874527524885, -0.5807223538112247, 0.5834419201245094], [0.0, 0.0, 0.0, 1.0]])
x_world_detection_position = np.array( [0.25, -0.8, 0.68])
mesh = trimesh.load_mesh("/volume/reconstruction_data/wink_do/stableGrasping/evals/grasping/ycb/b52a66ac-2738-4ece-9a12-1ac623996fd5_e8f314f6-916f-4ce2-9239-26dca11fd56d_no_filter/" + name + "/eval/textured.obj/0/mesh.obj")#_simplified0.003
if front:
    mesh = mesh.apply_translation([0, 0.15, 0])
    name += "_front"


table = trimesh.primitives.Box([1, 1, 0.6], trimesh.transformations.translation_matrix([0, 0, -0.3]))
mesh = trimesh.util.concatenate(mesh, table)
#mesh.show()

mesh = mesh.apply_translation(x_world_detection_position)
mesh.apply_transform(f_kinect_world)
print(mesh.bounds)
#mesh.show()

out_depth = np.zeros((480, 640), np.float32)
while True:
    begin = time.time()
    KinectSim.simulate(mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32), out_depth)
    print(time.time() - begin)
    break


#plt.imshow(out_depth)
#plt.show()

out_depth[out_depth == 0] = 2047

print(out_depth.min(), out_depth.max())

#out_depth *= 1000
width = 640
height = 480
rgbd_image = o3d.geometry.RGBDImage().create_from_color_and_depth(o3d.geometry.Image(np.repeat(out_depth[:, :, None], 3, -1).astype(np.uint8)),
                                                                o3d.geometry.Image(out_depth),
                                                                depth_scale=1.0,
                                                                depth_trunc=10.0,
                                                                convert_rgb_to_intensity=False)

intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx=width, fy=width, cx=width // 2, cy=height // 2)
extrinsic = np.identity(4)
#extrinsic = inv_trafo(pose)
#extrinsic[1, :] *= -1
#extrinsic[2, :] *= -1
pcd = o3d.geometry.PointCloud().create_from_rgbd_image(image=rgbd_image,
                                                    intrinsic=intrinsic,
                                                    extrinsic=extrinsic)

#points = project_depth(out_depth, depth_trunc=2)
#print(points.max(0), points.min(0))

#pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
o3d.visualization.draw_geometries([pcd, o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.05)])