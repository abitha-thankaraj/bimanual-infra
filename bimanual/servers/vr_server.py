import zmq
import time
import multiprocessing as mp
import numpy as np
from xarm import XArmAPI
from ctypes import c_double
import pandas as pd
from bimanual.hardware.robot.xarmrobot import  register_ctx, CustomManager, CartesianMoveMessage


x_max = 406
x_min = 206
y_max = 400
y_min = -400
z_max = 664
z_min = 470


SCALE_FACTOR = 1000
CONTROL_FREQ = 90.
CONTROL_TIME_PERIOD = 1/CONTROL_FREQ


def parse_pose_message(message):
    left_pose, right_pose = message.split('|')
    left_pose = [round(float(x),4) for x in left_pose.split(',')]
    right_pose = [round(float(x),4) for x in right_pose.split(',')]
    return left_pose, right_pose

# def start_server(queue):
#     # Set up the ZeroMQ context
#     context = zmq.Context()

#     # Create a REP (reply) socket
#     socket = context.socket(zmq.REP)
#     # Bind the socket to a specific address and port
#     socket.bind("tcp://*:5555")

#     start_teleop = False
#     init_pos_flag = False

#     rel_left_pose = [0. for  _ in range(7)]
#     rel_right_pose = [0. for  _ in range(7)]

#     last_ts = None


#     print("Starting server...")

#     while True:
#         # Wait for a message from the client
#         message = socket.recv_string()
#         # print("Received request: {}, at time:{}".format(message, time.time()))

#         if message.startswith("start"):
#             start_teleop = True
#             socket.send_string("OK: Starting teleop at time:{}".format(time.time()))
#             init_pos_flag = True
            
#         elif init_pos_flag == True:
#             init_left_pos, init_right_pos = parse_pose_message(message)
#             last_ts = time.time()
#             init_pos_flag = False
#             socket.send_string("OK: Received init pos")
            
#         elif start_teleop == True:
            
#             # Parse the message
#             _ , right_pose = parse_pose_message(message)

#             if time.time() - last_ts >= CONTROL_TIME_PERIOD:
#                 rel_right_pose[0] = (right_pose[0] - init_right_pos[0]) * SCALE_FACTOR
#                 rel_right_pose[1] = (right_pose[1] - init_right_pos[1]) * SCALE_FACTOR
            
#                 queue.put(CartesianMoveMessage(target=rel_right_pose[:3] + [0., 0., 0.], wait=False, relative=True)) #For now, only consider the position

#             socket.send_string("OK: {}".format(time.time()))



def start_server(left_queue, right_queue):
    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    socket = context.socket(zmq.REP)
    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    start_teleop = False
    init_left_pos, init_right_pos = None, None

    rel_left_pose = [0. for  _ in range(7)]
    rel_right_pose = [0. for  _ in range(7)]

    last_ts = None

    print("Starting server...")

    """
    Message format:
        1) Start teleop message : Used to initialize the teleop
        Description: 'start:l_x, l_y, l_z, l_rot_x, l_rot_y, l_rot_z, l_rot_w|r_x, r_y, r_z, r_rot_x, r_rot_y, r_rot_z, r_rot_w'
        Example: 'start:1.0, 2.0, 3.0, 45.0, 30.0, 60.0, 1.|1.0, 2.0, 3.0, 45.0, 30.0, 60.0, 1.'
        
        2) Stop teleop message: Used to stop the teleop
        'stop'

        3) Pose message: Used to send the pose of the controllers
        Description: 'l_x, l_y, l_z, l_rot_x, l_rot_y, l_rot_z, l_rot_w|r_x, r_y, r_z, r_rot_x, r_rot_y, r_rot_z, r_rot_w'
        Example: '1.0, 2.0, 3.0, 45.0, 30.0, 60.0, 1.|1.0, 2.0, 3.0, 45.0, 30.0, 60.0, 1.'

    """

    while True:
        # Wait for a message from the client
        message = socket.recv_string()
        # print("Received request: {}, at time:{}".format(message, time.time()))

        if message.startswith("start:"):
            start_teleop = True
            print(message)
            message = message[6:].strip() # Get rid of the "start" part. Extract pose info.         
            
            init_left_pos, init_right_pos = parse_pose_message(message)
            print("Initialize position: {}".format(message))
            last_ts = time.time()
            socket.send_string("OK: Starting teleop at time:{}; Initialized frames".format(last_ts))
        
        elif message.startswith("stop"):
            
            start_teleop = False
            socket.send_string("OK: Stopping teleop at time:{}".format(time.time()))
            print("Stopping teleop...")
            
            # Reset init frames to None
            init_left_pos, init_right_pos = None, None
            print("Resetting init frames to None")

        elif start_teleop == True:
            
            # Parse the message
            left_pose , right_pose = parse_pose_message(message)

            if time.time() - last_ts >= CONTROL_TIME_PERIOD:
                # WRT frame set by controller for now | robot to vr z->x, x->y, y->z
                rel_right_pose[1] = (right_pose[0] - init_right_pos[0]) * SCALE_FACTOR
                rel_right_pose[2] = (right_pose[1] - init_right_pos[1]) * SCALE_FACTOR
                rel_right_pose[0] = (right_pose[2] - init_right_pos[2]) * SCALE_FACTOR

                right_queue.put(CartesianMoveMessage(target=rel_right_pose[:3] + [0., 0., 0.], wait=False, relative=True)) #For now, only consider the position
                
                # WRT frame set by controller for now | robot to vr z->x, x->y, y->z
                rel_left_pose[1] = (left_pose[0] - init_left_pos[0]) * SCALE_FACTOR
                rel_left_pose[2] = (left_pose[1] - init_left_pos[1]) * SCALE_FACTOR
                rel_left_pose[0] = (left_pose[2] - init_left_pos[2]) * SCALE_FACTOR

                left_queue.put(CartesianMoveMessage(target=rel_left_pose[:3] + [0., 0., 0.], wait=False, relative=True)) #For now, only consider the position
                

            socket.send_string("OK: {}".format(time.time()))


def move_robot(queue, last_sent_msg_ts, control_timeperiod, ip):
    # Initialize xArm API
    arm = XArmAPI(ip)

    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=False)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    # arm.move_gohome(wait=True)
    # time.sleep(0.1)

    # arm.set_gripper_mode(0)
    # arm.set_gripper_enable(True)
    # arm.set_gripper_speed(5000)
    # arm.set_gripper_position(300, wait=True)

    home_pose_aa = [206.0, -0.0, 475, 180.00002, -0.0, 0.0]
    status = arm.set_position_aa(home_pose_aa, wait=True)
    assert status==0, "Failed to set robot at home position"

    status, home_pose = arm.get_position_aa()
    target_pose = [0]*6
    assert status==0, "Failed to get robot position"
    print("Robot position: {}".format(home_pose))
    
    # exit()
    arm.set_mode(1)
    arm.set_state(0)

    time.sleep(0.1)

    debug_id = 0
    df = pd.DataFrame(columns=['id', 'get_current_pose_time', 'current_pose', 'set_des_pose_time', 'des_pose'])
    debug_record = {
        'id' : debug_id, 
        'get_current_pose_time' : None,
        'current_pose' : None, 
        'set_des_pose_time': None, 
        'des_pose' : None
    }

    while True: 
        if (time.time() - last_sent_msg_ts) > control_timeperiod:

            if not queue.empty():
                move_msg = queue.get()

                if move_msg.is_terminal():
                    break

                for i in range(len(move_msg.target)):
                    target_pose[i] = move_msg.target[i] + home_pose[i]

                # ---------- Debug ----------- #
                # debug_id += 1 # may not need this; df has ordering
                # debug_record['id'] = debug_id
                # ---------------------------- #
                
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
                des_pose = (np.clip( delta_pose, -5, 5) + np.array( current_pose[:3])).tolist() + current_pose[3:]


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
                # ---------- Debug ----------- #
                # debug_record['set_des_pose_time'] = time.time()
                # debug_record['des_pose'] = des_pose
                
                # new_row = pd.DataFrame(debug_record)
                # df = pd.concat([df, new_row])
                # ---------------------------- #


                # x = arm.set_servo_cartesian_aa(move_msg.target, wait=False, relative=True, mvacc=50, speed=50)
                print("Robot set_servo_cartesian returns: {}".format(x))
                if x!=0:
                    print("Failed to set robot position")
                    break
                # x = arm.set_position(*move_msg.target, relative=True, wait=False)
                # print("Robot set_position returns: {}".format(x))

                last_sent_msg_ts = time.time()
                # c_double(time.time())
                # print("last_sent_msg: {}".format(last_sent_msg.target))

                # ---------- Debug ----------- #
                # print("Writing to debug.csv")
                # if debug_id % 200 == 0:
                #     df.to_csv('debug.csv', mode='w')
                # ---------------------------- #


            else:
                time.sleep(0.001)
        
        
        
if __name__ == "__main__":

    control_frequency = 90# TODO; measure control frequency for each box. COmmands should be sent at 30-250. If <30 - choppy; if>250 drop frames. Deatils in manual:
    control_timeperiod = 1./control_frequency


    register_ctx()
    ctx_manager = CustomManager(ctx=mp.get_context())
    ctx_manager.start()


    # last_sent_msg_ts = mp.Value("d", time.time(), lock=False) # Single process access; no need for lock; Makes process faster.
    # Last message sent to the message queue by the calling process
    last_sent_msg_ts = time.time() # Single process access; no need for lock; Makes process faster.
    last_queued_msg = ctx_manager.CartesianMoveMessage([0, 0, 0, 0, 0, 0], relative =True) # TODO : Implement this as mp object; use BaseManager;
    right_message_queue = mp.Queue()
    left_message_queue = mp.Queue()

    try:
        right_moving_process = mp.Process(target=move_robot, args=(right_message_queue, time.time(), control_timeperiod, "192.168.86.230"), name = "move_proc_right")
        left_moving_process = mp.Process(target=move_robot, args=(left_message_queue, time.time(), control_timeperiod, "192.168.86.216"), name = "move_proc_left") 
        start_server_process = mp.Process(target= start_server,args =(left_message_queue, right_message_queue,), name="server_proc")

        right_moving_process.start()
        left_moving_process.start()
        start_server_process.start()
    
    # keyboard interrupt exception
    except KeyboardInterrupt:
        right_moving_process.join()
        left_moving_process.join()
        start_server_process.join()
        ctx_manager.shutdown()

        print("Exiting...")
        exit()

    except Exception as e:
        print("Error: {}".format(e))
        exit()
