import numpy as np
from scipy.spatial.transform import Rotation as R


def inv_tf(translation: np.ndarray, rotation: np.ndarray):
    inv_rotation = rotation.T
    inv_translation = -1 * inv_rotation @ translation
    return inv_translation, inv_rotation


translation = np.array(
    [-2.9802322387695312e-08, -0.19999997317790985, -0.2999999523162842]
)  # (x, y, z)
rotation = R.from_quat(
    np.array([0.4999999701976776, -0.5, 0.5, 0.4999999701976776])
).as_matrix()  # (x, y, z, w)

translation, rotation = inv_tf(translation, rotation)

tf = np.zeros((4, 4), dtype=np.float64)
tf[:3, :3] = rotation
tf[:3, 3] = translation
tf[3, 3] = 1.0

np.savetxt(
    "tf.txt",
    tf,
    delimiter=",",
    newline=",\n",
    fmt="%.15f",
)
