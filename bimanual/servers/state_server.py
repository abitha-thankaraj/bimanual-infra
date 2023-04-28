import zmq
import time
import multiprocessing as mp
from state_message import parse_controller_state
from bimanual.hardware.robot.xarmrobot import GripperMoveMessage, CartesianMoveMessage
from bimanual.servers import CONTROL_FREQ, CONTROL_TIME_PERIOD, FLIP_MATRIX, SCALE_FACTOR, x_max, x_min, y_max, y_min, z_max, z_min

from scipy.spatial.transform import Rotation as R
import numpy as np
from bimanual.hardware.robot.xarmrobot import  register_ctx, CustomManager, CartesianMoveMessage
import pprint
from xarm import XArmAPI

def move_robot(queue, 
               last_sent_msg_ts, 
               control_timeperiod, 
               ip):
    # Initialize xArm API
    arm = XArmAPI(ip)

    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=False)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)

    home_pose_aa = [206.0, -0.0, 475, 180.00002, -0.0, 0.0]
    status = arm.set_position_aa(home_pose_aa, wait=True)
    assert status==0, "Failed to set robot at home position"

    status, home_pose = arm.get_position_aa()
    target_pose = [0]*6
    assert status==0, "Failed to get robot position"
    print("Robot position: {}".format(home_pose))
    
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(0.1)


    while True: 
        if (time.time() - last_sent_msg_ts) > control_timeperiod:

            if not queue.empty():
                move_msg = queue.get()

                if move_msg.is_terminal():
                    break

                # Only on position.
                for i in range(3):
                    target_pose[i] = move_msg.target[i] + home_pose[i]

                # Rotation - home pose
                target_rotation_matrix = move_msg.rotation @ R.as_matrix(R.from_rotvec(np.array(home_pose[3:]))) 
                target_rotation = np.degrees(R.from_matrix(target_rotation_matrix).as_rotvec()).tolist()
                
                
                if arm.has_err_warn:
                    arm.clean_error()
                    arm.clean_warn()
                    arm.motion_enable(enable=False)
                    arm.motion_enable(enable=True)
                    arm.set_mode(0)
                    arm.set_state(0)

                    status = arm.set_position_aa(home_pose_aa, wait=True)
                    assert status==0, "Failed to set robot at home position"
                    arm.set_mode(1)
                    arm.set_state(0)

                current_pose = arm.get_position_aa()[1]

                # ---------- Debug ----------- #
                # debug_record['get_current_pose_time'] = time.time()
                # debug_record['current_pose'] = current_pose
                # ---------------------------- #
                                
                delta_pose = np.array(target_pose[:3]) - np.array( current_pose[:3])
                des_pose = (np.clip( delta_pose, -5, 5) + np.array( current_pose[:3])).tolist() + target_rotation # How do you clip axes angles?
                # current_pose[3:]


                if des_pose[0] > x_max:
                    des_pose[0]=x_max
                elif des_pose[0] < x_min:
                    des_pose[0]=x_min
                if des_pose[1]>y_max:
                    des_pose[1]=y_max
                elif des_pose[1] < y_min:
                    des_pose[1]=y_min
                if des_pose[2]>z_max:
                    des_pose[2]=z_max
                elif des_pose[2] < z_min:
                    des_pose[2]=z_min
                
                print(des_pose)
                

                x = arm.set_servo_cartesian_aa( des_pose, wait=False, relative=False, mvacc=200, speed=50)
                print("Robot set_servo_cartesian returns: {}".format(x))
                if x!=0:
                    print("Failed to set robot position")
                    break


                last_sent_msg_ts = time.time()



            else:
                time.sleep(0.001)
        

def start_server(left_queue: mp.Queue, right_queue: mp.Queue):
    
    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    socket = context.socket(zmq.REP)
    
    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    start_teleop, gripper_closed_flag = False, False # Start with gripper open
    
    init_left_position, init_right_position = None, None
    init_left_rotation, init_right_rotation = None, None

    relative_left_position = np.array([0. for  _ in range(3)])
    relative_right_position = np.array([0. for  _ in range(3)])

    relative_left_rotation = np.eye(3)
    relative_right_rotation = np.eye(3)

    last_ts = None

    print("Starting server...")


    while True:
        # Wait for a message from the client
        message = socket.recv_string()
        
        # print(message)
        socket.send_string("OK: Received message")
        # Parse the message
        controller_state = parse_controller_state(message)

        pprint.pprint(controller_state)
        # Pressing A button calibrates first frame and starts teleop
        if controller_state.right_a:
            
            start_teleop = True
            print("Starting teleop...")
            last_ts = time.time()
            # TODO: Convert quaternion to axes angle
            init_left_position = controller_state.left_position
            init_left_rotation = controller_state.left_rotation_matrix
            
            init_right_position = controller_state.right_position
            init_right_rotation = controller_state.right_rotation_matrix

        
        # Pressing B button stops teleop. And resets calibration frames.
        if controller_state.right_b:
            print("Stopping teleop...")
            start_teleop = False
            init_left_position, init_right_position = None, None
            

        #TODO : Move toggle logic to robot.py; I want to set some lock/mutex such that
        #TODO:  Make this more responsive?

        # Index trigger to close; hand trigger to open.
        
        # if controller_state.left_index_trigger > 0.5:
        #     left_queue.put(GripperMoveMessage(0., wait = False)) 
        
        # if controller_state.left_hand_trigger > 0.5:
        #     left_queue.put(GripperMoveMessage(300, wait = False)) 

        # if controller_state.right_index_trigger > 0.5:
        #     right_queue.put(GripperMoveMessage(0., wait = False))
        
        # if controller_state.right_hand_trigger > 0.5:
        #     right_queue.put(GripperMoveMessage(300, wait = False)) 
        
        if start_teleop:
            # Insert teleop logic here.
            if time.time() - last_ts >= CONTROL_TIME_PERIOD:
                # WRT frame set by controller for now | robot to vr z->x, x->y, y->z
                relative_right_position = controller_state.right_position - init_right_position
                relative_right_rotation = controller_state.right_rotation_matrix @ np.linalg.pinv(init_right_rotation)

                right_queue.put(CartesianMoveMessage(target=list(relative_right_position[:3]) + [0., 0., 0.], rotation=relative_right_rotation, wait=False, relative=True)) #For now, only consider the position
                
                relative_left_position = controller_state.left_position - init_left_position
                relative_left_rotation = controller_state.left_rotation_matrix @ np.linalg.pinv(init_left_rotation)
                left_queue.put(CartesianMoveMessage(target=list(relative_left_position[:3]) + [0., 0., 0.], rotation=relative_left_rotation, wait=False, relative=True)) #For now, only consider the position
                
                last_ts = time.time()
        
if __name__ == "__main__":
    control_frequency = 90# TODO; measure control frequency for each box. COmmands should be sent at 30-250. If <30 - choppy; if>250 drop frames. Deatils in manual:
    control_timeperiod = 1./control_frequency


    register_ctx()
    ctx_manager = CustomManager(ctx=mp.get_context())
    ctx_manager.start()


    # last_sent_msg_ts = mp.Value("d", time.time(), lock=False) # Single process access; no need for lock; Makes process faster.
    # Last message sent to the message queue by the calling process
    last_sent_msg_ts = time.time() # Single process access; no need for lock; Makes process faster.
    # last_queued_msg = ctx_manager.CartesianMoveMessage([0, 0, 0, 0, 0, 0], relative =True) # TODO : Implement this as mp object; use BaseManager;
    right_message_queue = mp.Queue()
    left_message_queue = mp.Queue()

    try:
        right_moving_process = mp.Process(target = move_robot, args=(right_message_queue, time.time(), control_timeperiod, "192.168.86.230"), name = "move_proc_right")
        # left_moving_process = mp.Process(target=move_robot, args=(left_message_queue, time.time(), control_timeperiod, "192.168.86.216"), name = "move_proc_left") 
        start_server_process = mp.Process(target= start_server,args =(left_message_queue, right_message_queue,), name="server_proc")

        right_moving_process.start()
        # left_moving_process.start()
        start_server_process.start()
    
    # keyboard interrupt exception
    except KeyboardInterrupt:
        right_moving_process.join()
        # left_moving_process.join()
        start_server_process.join()
        ctx_manager.shutdown()

        print("Exiting...")
        exit()

    except Exception as e:
        print("Error: {}".format(e))
        exit()
