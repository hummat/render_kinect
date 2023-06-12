
from ctypes import *
import time
import numpy as np
import os
import trimesh
import matplotlib.pyplot as plt
import open3d as o3d

import pyrender
from pyrender.shader_program import ShaderProgramCache

lib = cdll.LoadLibrary("lib/libkinectSim.so")

lib.simulate.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32, ndim=2, flags='C_CONTIGUOUS'), c_int, np.ctypeslib.ndpointer(dtype=np.int32, ndim=2, flags='C_CONTIGUOUS'), c_int, np.ctypeslib.ndpointer(dtype=np.float32, ndim=2, flags='C_CONTIGUOUS')]
lib.simulate.restype = None

class KinectSim:

    @staticmethod
    def simulate(verts, faces, out_depth):
        lib.simulate(verts, len(verts), faces, len(faces), out_depth)
     

def render_kinect(mesh: o3d.geometry.TriangleMesh):        

    out_depth = np.zeros((480, 640), np.float32)
    KinectSim.simulate(mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32), out_depth)

    plt.subplots(figsize=(6, 4))
    plt.imshow(out_depth, cmap="jet")
    plt.axis("off")
    plt.tight_layout()
    plt.show()

    out_depth[out_depth == 0] = 2047

    print(out_depth.min(), out_depth.max())

    width = 640
    height = 480
    rgbd_image = o3d.geometry.RGBDImage().create_from_color_and_depth(o3d.geometry.Image(np.repeat(out_depth[:, :, None], 3, -1).astype(np.uint8)),
                                                                    o3d.geometry.Image(out_depth),
                                                                    depth_scale=1.0,
                                                                    depth_trunc=10.0,
                                                                    convert_rgb_to_intensity=False)

    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx=582.6989, fy=582.6989, cx=width // 2, cy=height // 2)
    extrinsic = np.identity(4)
    extrinsic[1, :] *= -1
    extrinsic[2, :] *= -1
    pcd = o3d.geometry.PointCloud().create_from_rgbd_image(image=rgbd_image,
                                                        intrinsic=intrinsic,
                                                        extrinsic=extrinsic)

    return np.asarray(pcd.points)


def render_perfect(mesh: o3d.geometry.TriangleMesh,
        width: int = 640,
        height: int = 480):
    trimesh_mesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.faces), process=False)
    pyrender_mesh = pyrender.Mesh.from_trimesh(trimesh_mesh, smooth=False)
    camera = pyrender.IntrinsicsCamera(fx=582.6989, fy=582.6989, cx=width // 2, cy=height // 2, znear=0.01, zfar=10.0)
    pose = np.eye(4)
    #pose[2, 3] = np.clip(np.random.normal(1, 0.4), 0.7, 1.5)

    scene = pyrender.Scene()
    scene.add(pyrender_mesh)
    scene.add(camera, pose=pose)

    renderer = pyrender.OffscreenRenderer(width, height)
    renderer._renderer._program_cache = ShaderProgramCache(shader_dir="shaders")

    normal_image, depth_image = renderer.render(scene, flags=pyrender.RenderFlags.SKIP_CULL_FACES)

    rgbd_image = o3d.geometry.RGBDImage().create_from_color_and_depth(o3d.geometry.Image(normal_image.astype(np.uint8)),
                                                                    o3d.geometry.Image(depth_image),
                                                                    depth_scale=1.0,
                                                                    depth_trunc=10.0,
                                                                    convert_rgb_to_intensity=False)

    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx=582.6989, fy=582.6989, cx=width // 2, cy=height // 2)
    extrinsic = np.eye(4)#inv_trafo(pose)
    extrinsic[1, :] *= -1
    extrinsic[2, :] *= -1
    pcd = o3d.geometry.PointCloud().create_from_rgbd_image(image=rgbd_image,
                                                        intrinsic=intrinsic,
                                                        extrinsic=extrinsic)

    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.colors) * 2 - 1
    return points, normals


f_kinect_world = np.array([[0.18889654314248513, -0.5491412049704937, 0.8141013875353555, -1.29464394785277],
                           [-0.9455020122657823, -0.325615105774899, -0.0002537971720279581, -0.009541282377521727],
                           [0.2652233842565174, -0.7696874527524885, -0.5807223538112247, 0.5834419201245094],
                           [0.0, 0.0, 0.0, 1.0]])
x_world_detection_position = np.array([0.25, -0.8, 0.68])
# mesh = trimesh.load_mesh("mesh_gray_bowl.obj")
mesh = trimesh.load_mesh("/run/media/matthias/2C20BCA320BC7604/datasets/automatica/029_gray_bowl/simplified.ply")

# Move a bit towards cam
mesh = mesh.apply_translation([0, 0.15, 0])

# Add table
table = trimesh.primitives.Box([1, 1, 0.6], trimesh.transformations.translation_matrix([0, 0, -0.3]))
print(table.vertices.shape, table.faces.shape)
mesh = trimesh.util.concatenate(mesh, table)

mesh = mesh.apply_translation(x_world_detection_position)
mesh.apply_transform(f_kinect_world)
print(mesh.bounds)
#mesh.show()

print(f_kinect_world)

points = render_kinect(mesh)
pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
o3d.io.write_point_cloud(str("kinect.pcd"), pcd)

o3d.visualization.draw_geometries([pcd, o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.05)])

# Pyrender uses -Z forward
mesh.apply_transform(trimesh.transformations.euler_matrix(np.pi, 0, 0))

points, _ = render_perfect(mesh)
pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
o3d.io.write_point_cloud(str("perfect.pcd"), pcd)

o3d.visualization.draw_geometries([pcd, o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.05)])
