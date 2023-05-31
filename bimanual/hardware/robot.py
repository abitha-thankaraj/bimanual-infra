import time
import numpy as np
import pandas as pd
from enum import Enum
import pybullet
import pybullet_data
import zmq

from typing import List
import multiprocessing as mp

from xarm import XArmAPI

from bimanual.servers.robot_state import RobotStateAction
from bimanual.utils.transforms import robot_pose_aa_to_affine, affine_to_robot_pose_aa
from bimanual.servers import CONTROL_TIME_PERIOD, ROBOT_WORKSPACE, ROBOT_HOME_JS, ROBOT_SERVO_MODE_STEP_LIMITS, DATA_DIR, RIGHT_ARM_IP, LEFT_ARM_IP


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

    def enable_impedance_mode(self):
        # set tool impedance parameters:
        # x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
        K_pos = 300
        # Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)
        K_ori = 4

        # Attention: for M and J, smaller value means less effort to drive the arm, but may also be less stable, please be careful.
        M = float(0.06)  # x/y/z equivalent mass; range: 0.02 ~ 1 kg
        # Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)
        J = M * 0.01

        c_axis = [1, 1, 1, 1, 1, 1]  # set z axis as compliant axis
        ref_frame = 0         # 0 : base , 1 : tool

        self.set_impedance_mbk([M, M, M, J, J, J], [K_pos, K_pos, K_pos, K_ori, K_ori, K_ori],
                               [0]*6)  # B(damping) is reserved, give zeros
        self.set_impedance_config(ref_frame, c_axis)

        # enable ft sensor communication
        self.ft_sensor_enable(1)
        # will overwrite previous sensor zero and payload configuration
        # remove this if zero_offset and payload already identified & compensated!
        self.ft_sensor_set_zero()
        # wait for writing zero operation to take effect, do not remove
        time.sleep(0.2)

        # move robot in impedance control application
        self.ft_sensor_app_set(1)
        # will start after set_state(0)
        self.set_state(0)

    def set_mode_and_state(self, mode: RobotControlMode, state: int = 0):
        self.set_mode(mode.value)
        self.set_state(state)
        self.set_gripper_mode(0)  # Gripper is always in position control.

    def reset(self):
        # Clean error
        self.clear()
        self.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        status = self.set_servo_angle_j(
            ROBOT_HOME_JS, wait=True, is_radian=True)
        # assert status == 0, "Failed to set robot at home joint position"
        time.sleep(0.1)

    def get_current_state_action_tuple(self,
                                       ts: float = None,
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
            pose_aa=np.array(self.get_position_aa()[
                             1]) if pose_aa is None else pose_aa,
            joint_angles=np.array(self.get_servo_angle(
            )[1]) if joint_angles is None else joint_angles,
            gripper_state=None,  # TODO: Make binary flag. This is too slow.
            # self.get_gripper_position(
            # )[1] if gripper_state is None else gripper_state,
            force_info=force_info,

            # Action information
            # Commanded pose; after clipping
            action_des_pose_aa=np.array(des_pose),
            # Master clock; Sync both arms with this.
            controller_ts=controller_ts,
            # Time at which the previous command was sent to the robot.
            last_sent_ts=last_sent_ts,
        )

# Define constants for communication
SERVER_PORT = 5555
CLIENT_PORT = 5555

def move_robot(queue: mp.Queue, ip: str, exit_event: mp.Event = None, traj_id: str = None):
    # Initialize ZeroMQ context and socket for client
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://localhost:{CLIENT_PORT}")
    robot = Robot(ip, is_radian=True)
    robot.reset()
    # if ip == "192.168.86.230":
    #     robot.enable_impedance_mode()
    #     print("Enabled impedance mode. Robot force {}".format(
    #         robot.get_ft_sensor_data()))
    #     robot.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)

    status, home_pose = robot.get_position_aa()
    assert status == 0, "Failed to get robot position"

    home_affine = robot_pose_aa_to_affine(home_pose)

    # Initialize timestamp; used to send messages to the robot at a fixed frequency.
    last_sent_msg_ts = time.time()

    # Initialize the environment state action tuple.
    env_state_action_df = robot.get_current_state_action_tuple(
        last_sent_ts=last_sent_msg_ts).to_df()

    while exit_event is None or not exit_event.is_set():
        if (time.time() - last_sent_msg_ts) > CONTROL_TIME_PERIOD:
            if not queue.empty():
                move_msg = queue.get()
                # TODO: Add df record
                print(f'Got queue message here: {move_msg}')
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

                # target_affine = home_affine @ move_msg.affine

                home_translation = home_affine[:3, 3]
                target_translation = home_affine[:3,
                                                 3] + move_msg.affine[:3, 3]

                home_rotation = home_affine[:3, :3]
                target_rotation = home_rotation @ move_msg.affine[:3, :3]

                target_affine = np.block(
                    [[target_rotation, target_translation.reshape(-1, 1)], [0, 0, 0, 1]])

                print("Target affine: {}".format(target_affine))
                # print("Robot force data: {}".format(
                #     robot.get_ft_sensor_data()))

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
                des_joint_angles = robot.get_inverse_kinematics(des_pose, input_is_radian=True, return_is_radian=True)[1]
                # Send request to collision detection server
                socket.send_multipart([ip.encode(), ','.join(str(item) for item in des_joint_angles).encode()])
                response = socket.recv().decode()
                print(response)
                if response == "True":
                    # Collision detected, skip and move to next command
                    continue
                ret_code, (joint_pos, joint_vels,
                           joint_effort) = robot.get_joint_states()
                # # # Populate all records for state.
                current_state_action_pair = robot.get_current_state_action_tuple(
                    # just use time.time? why use the last sent msg ts? Will this help get an exact state
                    ts=last_sent_msg_ts,
                    pose_aa=current_pose,
                    joint_angles=joint_pos,
                    des_pose=des_pose,
                    controller_ts=move_msg.created_timestamp,
                    last_sent_ts=last_sent_msg_ts  # t-1 actually;
                )

                env_state_action_df = pd.concat(
                    [env_state_action_df, current_state_action_pair.to_df()], ignore_index=True)

                # TODO: Get all the parameters from the message?
                robot.set_servo_cartesian_aa(
                    des_pose, wait=False, relative=False, mvacc=200, speed=50)

                last_sent_msg_ts = time.time()

            else:
                time.sleep(0.001)

    # Save the data to a file, when you exit the task.
    fname = "{}/{}/env_state_action_df_{}.csv".format(DATA_DIR, traj_id, ip)
    env_state_action_df.to_csv(fname, index=False)
    print("Saved robot data to file: {}".format(fname))
    # env_state_action_df.to_hdf(
    #     "/home/robotlab/projects/bimanual-infra/data/env_state_action_df_{}.h5".format(ip), key="df", mode="w")

    return

def start_simulation_with_zmq(exit_event: mp.Event = None):
    physicsClient = pybullet.connect(pybullet.GUI)

    # config GUI
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    # add the resource path
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    # set gravity
    pybullet.setGravity(0, 0, -9.8)

    # load scene
    planeID = pybullet.loadURDF('plane.urdf')

    # load robot
    robotStartPos = [0, 0, 0]
    robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])
    robotSpacing = 0.5  # Distance between robots
    robotID1 = pybullet.loadURDF("/hardware/model/robot.urdf", robotStartPos, robotStartOrientation)
    robotID2 = pybullet.loadURDF("/hardware/model/robot.urdf", [robotStartPos[0] + robotSpacing, robotStartPos[1], robotStartPos[2]], robotStartOrientation)
    #SET SIMULATION TO INITIAL POSITION
    for i in range(7):
                pybullet.setJointMotorControl2(robotID1, i, pybullet.POSITION_CONTROL, targetPosition=-ROBOT_HOME_JS[i])
                pybullet.setJointMotorControl2(robotID2, i, pybullet.POSITION_CONTROL, targetPosition=-ROBOT_HOME_JS[i])
    
    
    
    # start rendering
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    pybullet.setRealTimeSimulation(1)

    # Initialize ZeroMQ context and socket for server
    context = zmq.Context()
    socket = context.socket(zmq.ROUTER)
    socket.bind(f"tcp://*:{SERVER_PORT}")

    # Initialize robot states
    robot_states = {}
    robot_states[LEFT_ARM_IP] = ROBOT_HOME_JS
    robot_states[RIGHT_ARM_IP] = ROBOT_HOME_JS

    while exit_event is None or not exit_event.is_set():
        # Receive request from client, format is following:
        #[b'\x00k\x8bEg', b'', b'192.168.86.230', b'0.07235799729824066,-0.9553599953651428,-0.04017600044608116,0.6615110039710999,-0.03283600136637688,1.6164660453796387,0.04765599966049194']
        client_ip, _, robot_ip, des_angles = socket.recv_multipart() 
        robot_ip = robot_ip.decode()
        des_angles = [float(angle) for angle in des_angles.decode().split(',')]
        # Set robot state and perform collision detection
        robot_states[robot_ip] = des_angles
        for i in range(7):
                pybullet.setJointMotorControl2(robotID1, i, pybullet.POSITION_CONTROL, targetPosition=-robot_states[LEFT_ARM_IP][i])
                pybullet.setJointMotorControl2(robotID2, i, pybullet.POSITION_CONTROL, targetPosition=-robot_states[RIGHT_ARM_IP][i])
        contactPoints0 = pybullet.getContactPoints(robotID1, robotID2)

        # Send response to client indicating collision status
        socket.send_multipart([robot_ip.encode(), str(len(contactPoints0)>0).encode()]) #Here is probably sending wrong?
        print("Sent response to client: {}".format(len(contactPoints0)>0)) #up to here works fine. Robot_states dict is updating two robot infos.

    # Clean up ZeroMQ resources
    socket.close()
    context.term()
    # close server
    pybullet.disconnect()


