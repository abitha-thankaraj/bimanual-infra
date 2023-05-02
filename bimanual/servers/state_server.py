import zmq
import time
import numpy as np
import multiprocessing as mp

from bimanual.servers import CONTROL_TIME_PERIOD, FLIP_MATRIX, H_F
# , H_F
from bimanual.servers.controller_state import parse_controller_state
from bimanual.hardware.robot import CartesianMoveMessage, GripperMoveMessage

robot_home_affine = np.array([[ 1.00e+00, -2.21e-13, -1.27e-06,  2.06e-01],
 [ 2.21e-13, -1.00e+00,  3.46e-07, -6.60e-08],
 [-1.27e-06, -3.46e-07, -1.00e+00, 4.75e-01],
 [ 0.00e+00,  0.00e+00,  0.00e+00,  1.00e+00]])

def get_homogenous_inv(homogenous_matrix):
    """ Returns the inverse of a homogenous matrix. """
    R = homogenous_matrix[:3, :3]
    t = homogenous_matrix[:3, 3]
    inv_R = R.T
    inv_t = -R.T @ t
    return np.block([[inv_R, inv_t[:, np.newaxis]], [np.zeros((1, 3)), 1.]])


def start_server(left_queue: mp.Queue, right_queue: mp.Queue):
    np.set_printoptions(precision=2)
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

    # Last sent message timestamp. Used to control the frequency of messages sent to the queue.
    last_ts = None

    print("Starting server...")
    i = 0
    while True:
        # Wait for a message from the client
        message = socket.recv_string()

        # REP socket must send a reply back.
        socket.send_string("OK: Received message")

        controller_state = parse_controller_state(message)

        # Debug message
        # pprint.pprint(controller_state)

        # Pressing A button calibrates first frame and starts teleop
        if controller_state.right_a:

            start_teleop = True
            last_ts = time.time()
            init_left_affine, init_right_affine = controller_state.left_affine, controller_state.right_affine

        # Pressing B button stops teleop. And resets calibration frames to None.
        if controller_state.right_b:
            start_teleop = False
            init_left_affine, init_right_affine = None, None
            # Sentinal value to reset robot start pose to current pose.
            right_queue.put(CartesianMoveMessage(target=None))
            left_queue.put(CartesianMoveMessage(target=None))

        if start_teleop:
            # Why do you need a rate limiter here?
            # TODO: Check if this can be removed. You're dropping messages here. Does this make it smoother/ jerkier?

            # if time.time() - last_ts >= CONTROL_TIME_PERIOD:
                # end_affine = relative_affine @ start_affine
            i+=1
            
            # relative_right_affine = controller_state.right_affine @ np.linalg.pinv(init_right_affine)

            relative_right_affine = controller_state.right_affine @ get_homogenous_inv(init_right_affine)

            # K = robot_home_affine @ np.linalg.pinv(init_right_affine)

            # print(H_F @ relative_right_affine @ np.linalg.pinv(H_F))
            # H_R1_R0 = H_F @ relative_right_affine @ get_homogenous_inv(H_F)
            if i%20 == 0:
                print("Right affine; Aligned :\n {}".format(controller_state.right_affine))
            
                # print("Controller affine :\n {}".format(controller_state.right_affine))
                print("H_t :\n {}".format(relative_right_affine))

                # print("R_t :\n {}".format(H_t @ robot_home_affine))
                
                # print("Relative H :\n {}".format(relative_right_affine))
                # print(H_R1_R0 @ robot_home_affine @ np.linalg.pinv(init_right_affine))

            right_queue.put(CartesianMoveMessage(affine=relative_right_affine, target=[]))
            
            # right_queue.put(CartesianMoveMessage(affine=relative_right_affine, target=[]))

            relative_left_affine = controller_state.left_affine @ get_homogenous_inv(init_left_affine)
            left_queue.put(CartesianMoveMessage(affine=relative_left_affine, target=[]))

                # last_ts = time.time()

        # # Index trigger to close; hand trigger to open.
        # if controller_state.left_index_trigger > 0.5:
        #     left_queue.put(GripperMoveMessage(0., wait=False))

        # elif controller_state.left_hand_trigger > 0.5:
        #     # TODO: Move open to consts
        #     left_queue.put(GripperMoveMessage(300, wait=False))

        # if controller_state.right_index_trigger > 0.5:
        #     right_queue.put(GripperMoveMessage(0., wait=False))

        # elif controller_state.right_hand_trigger > 0.5:
        #     right_queue.put(GripperMoveMessage(300, wait=False))
