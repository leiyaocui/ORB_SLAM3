import os

import open3d as o3d

from config import DATA_PATH

mesh_path = os.path.join(DATA_PATH, "mesh.ply")

mesh = o3d.io.read_triangle_mesh(mesh_path)
o3d.visualization.draw_geometries([mesh])
