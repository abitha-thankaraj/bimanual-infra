import zmq
import time
import numpy as np
import multiprocessing as mp
from numpy.linalg import pinv

from bimanual.servers import H_R_V, H_R_V_star, GRIPPER_OPEN, GRIPPER_CLOSE
from bimanual.servers.controller_state import parse_controller_state
from bimanual.hardware.robot import CartesianMoveMessage, GripperMoveMessage


def get_relative_affine(init_affine, current_affine):
    """ Returns the relative affine from the initial affine to the current affine.
        Args:
            init_affine: Initial affine
            current_affine: Current affine
        Returns:
            Relative affine from init_affine to current_affine
    """
    # Relative affine from init_affine to current_affine in the VR controller frame.
    H_V_des = pinv(init_affine) @ current_affine

    # Transform to robot frame.
    # Flips axes
    relative_affine_rot = (pinv(H_R_V) @ H_V_des @ H_R_V)[:3, :3]
    # Translations flips are mirrored.
    relative_affine_trans = (pinv(H_R_V_star) @ H_V_des @ H_R_V_star)[:3, 3]

    # Homogeneous coordinates
    relative_affine = np.block(
        [[relative_affine_rot, relative_affine_trans.reshape(3, 1)], [0, 0, 0, 1]])

    return relative_affine


def start_subscriber(left_queue: mp.Queue, right_queue: mp.Queue, exit_event: mp.Event = None):
    """ Opens zmq socket to receive controller state messages from the oculus. 
        Controller states are parsed and sent as affines to shared message queus to be accessed by each xarm.
    """

    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a subscriber socket
    socket = context.socket(zmq.SUB)

    # Bind the socket to a specific address and port
    # IP address of oculus on NYU network.
    # TODO: Move to configs
    socket.connect("tcp://10.19.238.56:5555")

    # Subscribe to messages on topic "oculus_controller"
    # TODO: Move to configs
    socket.subscribe(b"oculus_controller")

    start_teleop = False

    start_teleop_left, start_teleop_right = False, False

    # Calibration frames
    init_left_affine, init_right_affine = None, None

    print("Starting Subscriber...")

    last_ts = time.time()

    while exit_event is None or not exit_event.is_set():
        # Subscribe to topic
        message = socket.recv_string()
        # TODO: Figure out why you get topic name and then message; may have something to do with zmq reading.
        if message == "oculus_controller":
            continue
        # print("Received msg")

        last_ts = time.time()

        controller_state = parse_controller_state(message)

        # Pressing B and Y exits the program.
        if controller_state.left_y and controller_state.right_b:
            # Cleanup zmq context
            socket.close()
            context.term()
            # Set the exit event to signal both arms to stop.
            exit_event.set()
            print("Exiting... Exit event is set")

            return

        # Teleop start/stop - right arm

        # Pressing A button calibrates first frame and starts teleop
        if controller_state.right_a:
            print('Starting teleop')
            start_teleop_right = True
            init_right_affine = controller_state.right_affine
            # start_teleop = True
            # init_left_affine, init_right_affine = controller_state.left_affine, controller_state.right_affine

        # Pressing B button stops teleop. And resets calibration frames to None.
        if controller_state.right_b:
            start_teleop_right = False
            init_right_affine = None

            # start_teleop = False
            # init_left_affine, init_right_affine = None, None
            # Sentinal value to reset robot start pose to current pose.
            right_queue.put(CartesianMoveMessage(target=None))
            # left_queue.put(CartesianMoveMessage(target=None))

        # Teleop start/stop - left arm
        if controller_state.left_x:
            start_teleop_left = True
            init_left_affine = controller_state.left_affine
        if controller_state.left_y:
            start_teleop_left = False
            init_left_affine = None
            left_queue.put(CartesianMoveMessage(target=None))

        if start_teleop_right:

            # LP edits
            # Relative rotation in controller from. From init_frame to current frame.

            relative_right_affine = get_relative_affine(
                init_right_affine, controller_state.right_affine)
            # relative_left_affine = get_relative_affine(
            #     init_left_affine, controller_state.left_affine)

            # Send relative affines to each arm.
            right_queue.put(CartesianMoveMessage(
                affine=relative_right_affine, target=[]))
            # left_queue.put(CartesianMoveMessage(
            #     affine=relative_left_affine, target=[]))

        if start_teleop_left:
            relative_left_affine = get_relative_affine(
                init_left_affine, controller_state.left_affine)
            left_queue.put(CartesianMoveMessage(
                affine=relative_left_affine, target=[]))

        # Index trigger to close; hand trigger to open.
        if controller_state.left_index_trigger > 0.5:
            left_queue.put(GripperMoveMessage(GRIPPER_CLOSE, wait=False))

        elif controller_state.left_hand_trigger > 0.5:
            left_queue.put(GripperMoveMessage(GRIPPER_OPEN, wait=False))

        if controller_state.right_index_trigger > 0.5:
            right_queue.put(GripperMoveMessage(GRIPPER_CLOSE, wait=False))

        elif controller_state.right_hand_trigger > 0.5:
            right_queue.put(GripperMoveMessage(GRIPPER_OPEN, wait=False))
