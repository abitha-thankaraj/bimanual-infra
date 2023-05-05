import time
import numpy as np
import pandas as pd
from enum import Enum

from typing import List
import multiprocessing as mp

from xarm import XArmAPI

from bimanual.servers.robot_state import RobotStateAction
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
        super(Robot, self).__init__(
            port=ip, is_radian=is_radian, is_tool_coord=False)
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

    def get_current_state_action_tuple(self,
                                       pose_aa: np.ndarray = None,
                                       joint_angles: np.ndarray = None,
                                       gripper_state: np.ndarray = None,
                                       force_info: np.ndarray = None,
                                       des_pose: np.ndarray = None,
                                       controller_ts: float = None,
                                       last_sent_ts: float = None) -> RobotStateAction:
        # TODO: Add check for error code in get commands.
        # Refactor this to decouple state and action in the environment.
        return RobotStateAction(
            created_timestamp=time.time(),  # Time at which row was created.

            # state information
            pose_aa=self.get_position_aa()[1] if pose_aa is None else pose_aa,
            joint_angles=self.get_servo_angle(
            )[1] if joint_angles is None else joint_angles,
            gripper_state=self.get_gripper_position(
            )[1] if gripper_state is None else gripper_state,
            force_info=force_info,  # TODO: Bobby

            # Action information
            action_des_pose_aa=des_pose,  # Commanded pose; after clipping
            # Master clock; Sync both arms with this.
            controller_ts=controller_ts,
            # Time at which the previous command was sent to the robot.
            last_sent_ts=last_sent_ts,
        )


def move_robot(queue: mp.Queue, ip: str, exit_event: mp.Event = None):

    robot = Robot(ip, is_radian=True)
    robot.reset()

    status, home_pose = robot.get_position_aa()
    assert status == 0, "Failed to get robot position"

    home_affine = robot_pose_aa_to_affine(home_pose)
    env_state_action_df = robot.get_current_state_action_tuple().to_df()

    # Initialize timestamp; used to send messages to the robot at a fixed frequency.
    last_sent_msg_ts = time.time()

    while exit_event is None or not exit_event.is_set():
        if (time.time() - last_sent_msg_ts) > CONTROL_TIME_PERIOD:
            if not queue.empty():
                move_msg = queue.get()
                # TODO: Add df record

                if isinstance(move_msg, GripperMoveMessage):
                    robot.set_gripper_position(
                        move_msg.target, wait=move_msg.wait)
                    continue

                # B button pressed. When the robot stops; that becomes the new init frame.
                if move_msg.is_terminal:
                    home_pose = robot.get_position_aa()[1]  # translation in mm
                    home_affine = robot_pose_aa_to_affine(
                        home_pose)  # translation in m
                    print("Pausing robot at : {}".format(home_pose))
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
                delta_translation = np.array(
                    target_pose[:3]) - np.array(current_pose[:3])

                # When using servo commands, the maximum distance the robot can move is 10mm; clip translations accordingly.
                delta_translation = np.clip(delta_translation,
                                            a_min=ROBOT_SERVO_MODE_STEP_LIMITS[0],
                                            a_max=ROBOT_SERVO_MODE_STEP_LIMITS[1])

                # a_min and a_max are the boundaries of the robot's workspace; clip absolute position to these boundaries.

                des_translation = delta_translation + \
                    np.array(current_pose[:3])
                des_translation = np.clip(des_translation,
                                          a_min=ROBOT_WORKSPACE[0],
                                          a_max=ROBOT_WORKSPACE[1]).tolist()

                des_rotation = target_pose[3:]
                des_pose = des_translation + des_rotation

                # Populate all records for state.
                current_state_action_pair = robot.get_current_state_action_tuple(
                    # just use time.time? why use the last sent msg ts? Will this help get an exact state
                    ts=last_sent_msg_ts,
                    pose_aa=current_pose,
                    des_pose=des_pose,
                    controller_ts=move_msg.controller_ts,
                    last_sent_ts=last_sent_msg_ts  # t-1 actually;
                )

                env_state_action_df = pd.concat(
                    [env_state_action_df, current_state_action_pair.to_df()])

                # TODO: Get all the parameters from the message?
                robot.set_servo_cartesian_aa(
                    des_pose, wait=False, relative=False, mvacc=200, speed=50)

                last_sent_msg_ts = time.time()

            else:
                time.sleep(0.001)

    # Save the data to a file, when you exit the task.
    env_state_action_df.to_csv(
        "/home/robotlab/projects/bimanual-infra/data/env_state_action_df_{}.csv".format(ip), index=False)

    return



