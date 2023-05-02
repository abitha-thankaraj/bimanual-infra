import time
import socket
import numpy as np
from enum import Enum
from xarm import XArmAPI
import multiprocessing as mp
from typing import List

from bimanual.utils.transforms import robot_pose_aa_to_affine, affine_to_robot_pose_aa
from bimanual.servers import CONTROL_TIME_PERIOD, ROBOT_WORKSPACE, ROBOT_HOME_POSE_AA, ROBOT_SERVO_MODE_STEP_LIMITS


class RobotControlMode(Enum):
    CARTESIAN_CONTROL = 0
    SERVO_CONTROL = 1


class MoveMessage:
    def __init__(self, target: List[float]):
        self.target = target
        self.created_timestamp = time.time()

    @property
    def is_terminal(self) -> bool:  # Sentinal message
        return self.target is None


class CartesianMoveMessage(MoveMessage):
    def __init__(self, target: List[float], speed: float = 50., acceleration: float = 200.,
                 relative: bool = False, wait: bool = True, is_radian: bool = True,
                 affine: np.ndarray = None):
        super(CartesianMoveMessage, self).__init__(target)
        self.speed = speed
        self.mvacc = acceleration
        self.relative = relative
        # Default to async calls; do not wait for robot to finish moving.
        self.wait = wait
        self.is_radian = is_radian
        self.affine = affine


class GripperMoveMessage(MoveMessage):
    def __init__(self, target, wait=False):
        super(GripperMoveMessage, self).__init__(target)
        self.wait = wait


class Robot(XArmAPI):
    def __init__(self, ip="192.168.86.230", is_radian=True):
        super(Robot, self).__init__(port=ip, is_radian=is_radian, is_tool_coord=False)
        self.set_gripper_enable(True)

    def clear(self):
        self.clean_error()
        self.clean_warn()
        self.motion_enable(enable=False)
        self.motion_enable(enable=True)

    def set_mode_and_state(self, mode: RobotControlMode, state: int = 0):
        self.set_mode(mode.value)
        self.set_state(state)
        self.set_gripper_mode(0)  # Gripper is always in position control.
        
    def read_force_data(self):
        self.motion_enable(enable=True)
        self.ft_sensor_enable(0)
        self.clear()
        self.ft_sensor_enable(1)
        time.sleep(0.5)
        self.ft_sensor_set_zero()
        while self.connected and self.error_code == 0:
            print('raw_force: {}'.format(self.ft_raw_force))
            print('exe_force: {}'.format(self.ft_ext_force))
            time.sleep(0.2)

    def reset(self):
        # Clean error
        self.clear()
        self.set_mode_and_state(RobotControlMode.CARTESIAN_CONTROL, 0)
        # Move to predefined home position using cartesian control
        # TODO: Get joint states and set them. Deterministic.
        status = self.set_position_aa(ROBOT_HOME_POSE_AA, wait=True)
        assert status == 0, "Failed to set robot at home position"
        # Set mode to servo control
        self.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        # Wait for mode switch to complete
        time.sleep(0.1)


def move_robot(queue: mp.Queue, ip: str):

    robot = Robot(ip, is_radian=True)

    # Initialize timestamp; used to send messages to the robot at a fixed frequency.
    last_sent_msg_ts = time.time()
    robot.reset()

    status, home_pose = robot.get_position_aa()
    assert status == 0, "Failed to get robot position"

    home_affine = robot_pose_aa_to_affine(home_pose)

    while True:
        if (time.time() - last_sent_msg_ts) > CONTROL_TIME_PERIOD:
            if not queue.empty():
                move_msg = queue.get()

                if isinstance(move_msg, GripperMoveMessage):
                    robot.set_gripper_position(move_msg.target, wait=move_msg.wait)
                    continue

                # B button pressed. When the robot stops; that becomes the new init frame.
                if move_msg.is_terminal:
                    home_pose = robot.get_position_aa()[1]  # translation in mm
                    home_affine = robot_pose_aa_to_affine(home_pose)  # translation in m
                    print("Resetting : {}".format(home_pose))
                    continue

                # If robot is in error state, clear it. Reset to home. Consume next message.
                if robot.has_err_warn:
                    robot.reset()
                    home_pose = robot.get_position_aa()[1]
                    home_affine = robot_pose_aa_to_affine(home_pose)
                    continue

                target_affine = home_affine @ move_msg.affine
                print("Target affine: {}".format(target_affine))

                # If this target pose is too far from the current pose, move it to the closest point on the boundary.
                target_pose = affine_to_robot_pose_aa(target_affine).tolist()
                current_pose = robot.get_position_aa()[1]
                delta_translation = np.array(target_pose[:3]) - np.array(current_pose[:3])

                # When using servo commands, the maximum distance the robot can move is 10mm; clip translations accordingly.
                delta_translation = np.clip(delta_translation,
                                            a_min=ROBOT_SERVO_MODE_STEP_LIMITS[0],
                                            a_max=ROBOT_SERVO_MODE_STEP_LIMITS[1])

                # a_min and a_max are the boundaries of the robot's workspace; clip absolute position to these boundaries.

                des_translation = delta_translation + np.array(current_pose[:3])
                des_translation = np.clip(des_translation, 
                                          a_min=ROBOT_WORKSPACE[0], 
                                          a_max=ROBOT_WORKSPACE[1]).tolist()

                des_rotation = target_pose[3:]
                des_pose = des_translation + des_rotation

                # TODO: Get all the parameters from the message?
                robot.set_servo_cartesian_aa(des_pose, wait=False, relative=False, mvacc=200, speed=50)

                last_sent_msg_ts = time.time()

            else:
                time.sleep(0.001)
