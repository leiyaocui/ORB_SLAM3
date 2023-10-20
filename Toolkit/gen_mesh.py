import os
from glob import glob

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm

from config import DATA_PATH

input_path = DATA_PATH
trajectory_path = os.path.join(input_path, "tf_pred.txt")
color_paths = sorted(glob(os.path.join(input_path, "rgb", "*.png")))
depth_paths = sorted(glob(os.path.join(input_path, "depth", "*.png")))
output_path = os.path.join(input_path, "mesh.ply")
intrinsic = o3d.camera.PinholeCameraIntrinsic(
    1280, 720, 1038.4156494140625, 800.4813232421875, 640.0, 360.0
)


class CameraPose:
    def __init__(self, meta, mat, t):
        self.metastr = meta
        self.pose = mat
        self.timestamp = t


def read_trajectory(filename):
    # TUM format
    traj = []
    for line in open(filename, "r"):
        metastr = line.strip()
        metadata = list(map(float, metastr.split(" ")))

        timestamp = metadata[0]
        translation = metadata[1:4]  # (x, y, z)
        rotation = metadata[4:]  # (qx, qy, qz, qw)

        translation = np.array(translation)
        rotation = R.from_quat(np.array(rotation)).as_matrix()

        pose = np.zeros((4, 4))
        pose[:3, :3] = rotation
        pose[:3, 3] = translation
        pose[3, 3] = 1.0

        traj.append(CameraPose(metadata, pose, timestamp))
    return traj


camera_poses = read_trajectory(trajectory_path)

volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=4.0 / 512.0,
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
)

for i in tqdm(range(len(camera_poses)), dynamic_ncols=True):
    color = o3d.io.read_image(color_paths[i])
    depth = o3d.io.read_image(depth_paths[i])
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_trunc=40.0, depth_scale=1000, convert_rgb_to_intensity=False
    )
    volume.integrate(
        rgbd,
        intrinsic,
        np.linalg.inv(camera_poses[i].pose),
    )

print("Extract a triangle mesh from the volume")
mesh = volume.extract_triangle_mesh()
mesh.compute_vertex_normals()
o3d.io.write_triangle_mesh(output_path, mesh)
