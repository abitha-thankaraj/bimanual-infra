import zmq
import numpy as np
import multiprocessing as mp
from numpy.linalg import pinv

from bimanual.servers import H_R_V
from bimanual.servers.controller_state import parse_controller_state
from bimanual.hardware.robot import CartesianMoveMessage, GripperMoveMessage


def start_server(left_queue: mp.Queue, right_queue: mp.Queue):
    """ Opens zmq socket to receive controller state messages from the oculus. 
        Controller states are parsed and sent as affines to shared message queus to be accessed by each xarm.
    """

    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    socket = context.socket(zmq.REP)

    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    start_teleop = False

    # Calibration frames
    init_left_affine, init_right_affine = None, None

    print("Starting server...")

    while True:
        # Wait for a message from the client
        message = socket.recv_string()

        # REP socket must send a reply back.
        socket.send_string("OK: Received message")

        controller_state = parse_controller_state(message)

        # Pressing A button calibrates first frame and starts teleop
        if controller_state.right_a:

            start_teleop = True
            init_left_affine, init_right_affine = controller_state.left_affine, controller_state.right_affine

        # Pressing B button stops teleop. And resets calibration frames to None.
        if controller_state.right_b:
            start_teleop = False
            init_left_affine, init_right_affine = None, None
            # Sentinal value to reset robot start pose to current pose.
            right_queue.put(CartesianMoveMessage(target=None))
            left_queue.put(CartesianMoveMessage(target=None))

        if start_teleop:

            # LP edits
            # Relative rotation in controller from. From init_frame to current frame.
            H_VR_des = pinv(init_right_affine) @ controller_state.right_affine
            H_VL_des = pinv(init_left_affine) @ controller_state.left_affine

            # Align axes. Flip and rotate.
            relative_right_affine = pinv(H_R_V) @ H_VR_des @ H_R_V
            relative_left_affine = pinv(H_R_V) @ H_VL_des @ H_R_V

            # Send relative affines to each arm.
            right_queue.put(CartesianMoveMessage(affine=relative_right_affine, target=[]))
            left_queue.put(CartesianMoveMessage(affine=relative_left_affine, target=[]))

        # Index trigger to close; hand trigger to open.
        if controller_state.left_index_trigger > 0.5:
            left_queue.put(GripperMoveMessage(0., wait=False))

        elif controller_state.left_hand_trigger > 0.5:
            left_queue.put(GripperMoveMessage(400, wait=False))

        if controller_state.right_index_trigger > 0.5:
            right_queue.put(GripperMoveMessage(0., wait=False))

        elif controller_state.right_hand_trigger > 0.5:
            right_queue.put(GripperMoveMessage(400, wait=False))