import numpy as np
# import pandas as pd
from typing import Tuple
from dataclasses import dataclass

from bimanual.servers.state import State

@dataclass
class RobotState(State):
    """ Dataclass for storing the state of the robot. """
    # Robot pose wrt base frame (translation in mm, rotation in axis-angle representation)
    pose_aa: np.ndarray[Tuple[float, float, float, float, float, float]]
    # Joint angles (in radians)
    joint_angles: np.ndarray[Tuple[float, float, float, float, float, float, float]]
    # Gripper open width (in mm)
    gripper_state: float  # TODO: Should this be binary? Open/closed; bool?
    # Force information (in ? units) # TODO: Bobby
    force_info: float  # TODO: Bobby
