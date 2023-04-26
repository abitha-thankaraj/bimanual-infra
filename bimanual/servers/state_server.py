import zmq
import time
from state_message import parse_controller_state
from bimanual.hardware.robot.xarmrobot import GripperMoveMessage, CartesianMoveMessage


# TODO: Move this to init file
x_max = 406
x_min = 206
y_max = 200
y_min = -200
z_max = 420
z_min = 280


SCALE_FACTOR = 1000
CONTROL_FREQ = 90.
CONTROL_TIME_PERIOD = 1/CONTROL_FREQ



def start_server(left_queue, right_queue):
    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    
    socket = context.socket(zmq.REP)
    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    start_teleop, gripper_closed_flag = False, False # Start with gripper open
    init_left_pose, init_right_pose = None, None

    relative_left_pose = [0. for  _ in range(7)]
    relative_right_pose = [0. for  _ in range(7)]

    last_ts = None

    print("Starting server...")


    while True:
        # Wait for a message from the client
        message = socket.recv_string()
        
        # Parse the message
        controller_state = parse_controller_state(message)

        # Pressing A button calibrates first frame and starts teleop
        if controller_state.right_a:
            
            start_teleop = True
            print("Starting teleop...")
            last_ts = time.time()
            # TODO: Convert quaternion to axes angle
            init_left_pose = controller_state.left_local_position + controller_state.left_local_rotation
            init_right_pose = controller_state.right_local_position + controller_state.right_local_rotation

            socket.send_string("OK: Starting teleop at time:{}; Initialized frames".format(last_ts))
        
        # Pressing B button stops teleop. And resets calibration frames.
        elif controller_state.right_b:
            start_teleop = False
            init_left_pose, init_right_pose = None, None
            socket.send_string("OK: Stopping teleop at time:{}".format(time.time()))
        
        
        if controller_state.left_index_trigger > 0.5:
            # close?
            left_queue.put(GripperMoveMessage(0., wait = False)) 
            #TODO : Move toggle logic to robot.py; I want to set some lock/mutex such that
        if controller_state.left_hand_trigger > 0.5:
            # open?
            left_queue.put(GripperMoveMessage(300, wait = False)) #TODO : Move toggle logic to robot.py

        if controller_state.right_index_trigger > 0.5:
            # close?
            right_queue.put(GripperMoveMessage(0., wait = False)) #TODO : Move toggle logic to robot.py
        if controller_state.right_hand_trigger > 0.5:
            # open?
            right_queue.put(GripperMoveMessage(300, wait = False)) #TODO : Move toggle logic to robot.py

        if start_teleop:

            # Insert teleop logic here.
            if time.time() - last_ts >= CONTROL_TIME_PERIOD:
                right_pose = controller_state.right_local_position + controller_state.right_local_rotation
                left_pose = controller_state.left_local_position + controller_state.left_local_rotation
                
                # WRT frame set by controller for now | robot to vr z->x, x->y, y->z
                relative_right_pose[1] = (right_pose[0] - init_right_pose[0]) * SCALE_FACTOR
                relative_right_pose[2] = (right_pose[1] - init_right_pose[1]) * SCALE_FACTOR
                relative_right_pose[0] = (right_pose[2] - init_right_pose[2]) * SCALE_FACTOR

                right_queue.put(CartesianMoveMessage(target=relative_right_pose[:3] + [0., 0., 0.], wait=False, relative=True)) #For now, only consider the position
                
                # WRT frame set by controller for now | robot to vr z->x, x->y, y->z
                relative_left_pose[1] = (left_pose[0] - init_left_pose[0]) * SCALE_FACTOR
                relative_left_pose[2] = (left_pose[1] - init_left_pose[1]) * SCALE_FACTOR
                relative_left_pose[0] = (left_pose[2] - init_left_pose[2]) * SCALE_FACTOR

                left_queue.put(CartesianMoveMessage(target=relative_left_pose[:3] + [0., 0., 0.], wait=False, relative=True)) #For now, only consider the position
                
                last_ts = time.time()
            socket.send_string("OK: {}".format(time.time()))
        
