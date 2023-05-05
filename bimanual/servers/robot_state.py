import numpy as np
from typing import Tuple
from dataclasses import dataclass

from bimanual.servers.state import State


@dataclass
class RobotStateAction(State):
    """ Dataclass for storing the state of the robot. """
    # State information
    # Robot pose wrt base frame (translation in mm, rotation in axis-angle representation)
    pose_aa: np.ndarray[Tuple[float, float, float, float, float, float]]
    # Joint angles (in radians)
    joint_angles: np.ndarray[Tuple[float, float, float, float, float, float, float]]
    # Gripper open width (in mm)
    gripper_state: float  # TODO: Should this be binary? Open/closed; bool?
    # Force information (in ? units) # TODO: Bobby
    force_info: float  # TODO: Bobby
    # Action information
    # Desired position; Commanded action
    action_des_pose_aa: np.ndarray[Tuple[float, float, float, float, float, float]]

    # Time at which the controller state was transmitted
    controller_ts: float # This shoulkd be used only for ordering?
    # Time at which the robot state was commanded?
    last_sent_ts: float # Shoudl this be used for syncing the obses??? TODO: Check this again.
    

