import time
import numpy as np
from enum import Enum
from xarm import XArmAPI
import multiprocessing as mp

from bimanual.servers import CONTROL_TIME_PERIOD, ROBOT_WORKSPACE, ROBOT_HOME_POSE_AA, ROBOT_SERVO_MODE_STEP_LIMITS
from bimanual.utils.transforms import robot_pose_aa_to_affine, affine_to_robot_pose_aa


class RobotControlMode(Enum):
    CARTESIAN_CONTROL = 0
    SERVO_CONTROL = 1

class MoveMessage:
    def __init__(self, target):
        self.target = target
        self.created_timestamp = time.time()

    def is_terminal(self):  # Sentinal message
        return self.target is None


class CartesianMoveMessage(MoveMessage):
    def __init__(
        self,
        target,
        speed=50,
        acceleration=200,
        relative=False,
        wait=True,
        is_radian=True,
        affine=None,
    ):
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
        

    def set_mode_and_state(self, mode: RobotControlMode, state: int=0):
        self.set_mode(mode.value)
        self.set_state(state)
        self.set_gripper_mode(0)

    def reset(self):
        # Clean error
        self.clear()
        self.set_mode_and_state(RobotControlMode.CARTESIAN_CONTROL, 0)
        # Move to predefined home position using cartesian control
        status = self.set_position_aa(ROBOT_HOME_POSE_AA, wait=True) #TODO: Get joint states and set them. Derterministic.
        assert status == 0, "Failed to set robot at home position"
        # Set mode to servo control
        self.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        # Wait for mode switch to complete
        time.sleep(0.1)

        

def move_robot(queue: mp.Queue, ip: str):
    np.set_printoptions(precision=3, suppress=True)
    # Initialize xArm API
    robot = Robot(ip, is_radian=True)

    # Initialize timestamp; used to send messages to the robot at a fixed frequency.
    last_sent_msg_ts = time.time()
    robot.reset()

    status, home_pose = robot.get_position_aa()
    assert status == 0, "Failed to get robot position"
    # print("Robot position: {}".format(home_pose))

    home_affine = robot_pose_aa_to_affine(home_pose)
    # print("Home affine: {}".format(home_affine))


    while True:
        if (time.time() - last_sent_msg_ts) > CONTROL_TIME_PERIOD:
            if not queue.empty():
                move_msg = queue.get()

                if type(move_msg).__name__ == "GripperMoveMessage":
                    robot.set_gripper_position(move_msg.target, wait=move_msg.wait)
                    continue

                # B button pressed. When the robot stops; that becomes the new init frame.
                if move_msg.is_terminal():
                    home_pose = robot.get_position_aa()[1] # translation in mm
                    home_affine = robot_pose_aa_to_affine(home_pose) # translation in m
                    print("Resetting : {}".format(home_pose))
                    continue

                # If robot is in error state, clear it. Reset to home. Consume next message.
                if robot.has_err_warn:
                    robot.reset()
                    home_pose = robot.get_position_aa()[1]
                    home_affine = robot_pose_aa_to_affine(home_pose)
                    continue

                # end_affine =  relative_affine @ start_affine

                # print("Move message - relative affine: {}".format(move_msg.affine))
                # print("Home affine: {}".format(home_affine))

                target_affine = home_affine @ move_msg.affine 
                print("Target affine: {}".format(target_affine))
                
                target_pose = affine_to_robot_pose_aa(target_affine).tolist() # Translation in mm
                # print("Target pose: {}".format(target_pose))

                # If this target pose is too far from the current pose, move it to the closest point on the boundary.

                current_pose = robot.get_position_aa()[1]

                # When using servo commands, the maximum distance the robot can move is 10mm; clip translations accordingly.
                # -5, 5 is a loose safety margin; should be fine for now.
                # TODO: Make this a parameter. + smaller!

                delta_translation = np.array(target_pose[:3]) - np.array(current_pose[:3])
                # print("Delta translation: {}".format(delta_translation))
                delta_translation = np.clip(delta_translation, 
                                            a_min = ROBOT_SERVO_MODE_STEP_LIMITS[0], 
                                            a_max = ROBOT_SERVO_MODE_STEP_LIMITS[1])
                # print("Delta translation clipped: {}".format(delta_translation))
                # a_min and a_max are the boundaries of the robot's workspace; clip absolute position to these boundaries.

                # print("Current position: {}".format(current_pose[:3]))
                des_translation = delta_translation + np.array(current_pose[:3])
                # print("Des translation: {}".format(des_translation))
                des_translation = np.clip(des_translation, a_min=ROBOT_WORKSPACE[0], a_max=ROBOT_WORKSPACE[1]).tolist()

                # print("Des translation clipped: {}".format(des_translation))

                # TODO: How do you clip axes angles? (to quaternions and scale?) Do you need to?
                des_rotation = target_pose[3:]

                des_pose = des_translation + des_rotation
                # print("Des pose: {}".format(des_pose))

                # exit()
                
				#TODO: Get all the parameters from the message?
                x = robot.set_servo_cartesian_aa(
                    des_pose, wait=False, relative=False, mvacc=200, speed=50
                )

                # assert x == 0, "Failed to set robot position: return code {}".format(
                #     x)

                last_sent_msg_ts = time.time()

            else:
                time.sleep(0.001)
