import numpy as np
from scipy.spatial.transform import Rotation as R
# Scale factor is used to convert from m to mm and mm to m.
from bimanual.servers import SCALE_FACTOR


def robot_pose_aa_to_affine(pose_aa: np.ndarray) -> np.ndarray:
    """Converts a robot pose in axis-angle format to an affine matrix.
    Args:
        pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
        x, y, z are in mm and ax, ay, az are in radians.
    Returns:
        np.ndarray: 4x4 affine matrix [[R, t],[0, 1]]
    """
    # print("Robot pose aa: ", pose_aa)

    rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
    translation = np.array(pose_aa[:3]) / SCALE_FACTOR

    return np.block([[rotation, translation[:, np.newaxis]],
                     [0, 0, 0, 1]])


def affine_to_robot_pose_aa(affine: np.ndarray) -> np.ndarray:
    """Converts an affine matrix to a robot pose in axis-angle format.
    Args:
        affine (np.ndarray): 4x4 affine matrix [[R, t],[0, 1]]
    Returns:
        list: [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
        x, y, z are in mm and ax, ay, az are in radians.
    """
    translation = affine[:3, 3] * SCALE_FACTOR
    rotation = R.from_matrix(affine[:3, :3]).as_rotvec()
    return np.concatenate([translation, rotation])
