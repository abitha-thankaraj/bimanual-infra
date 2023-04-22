import zmq
import time
import multiprocessing as mp
import numpy as np
from bimanual.hardware.robot.xarmrobot import CartesianMoveMessage

SCALE_FACTOR = 1000
CONTROL_FREQ = 90.
CONTROL_TIME_PERIOD = 1/CONTROL_FREQ


def parse_pose_message(message):
    left_pose, right_pose = message.split('|')
    left_pose = [round(float(x),3) for x in left_pose.split(',')]
    right_pose = [round(float(x),3) for x in right_pose.split(',')]
    return left_pose, right_pose

def start_server(queue):
    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    socket = context.socket(zmq.REP)
    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    start_teleop = False
    prev_left_pos, prev_right_pos = None, None
    init_pos = False

    rel_left_pose = [0. for  _ in range(7)]
    rel_right_pose = [0. for  _ in range(7)]

    last_ts = None


    print("Starting server...")

    while True:
        # Wait for a message from the client
        message = socket.recv_string()
        # print("Received request: {}, at time:{}".format(message, time.time()))

        if message.startswith("start"):
            start_teleop = True
            socket.send_string("OK: Starting teleop at time:{}".format(time.time()))
            init_pos = True
            continue    
        
        elif init_pos:
            prev_left_pos, prev_right_pos = parse_pose_message(message)
            last_ts = time.time()
            init_pos = False
            socket.send_string("OK: Received init pos")
            continue
        
        elif prev_left_pos is not None and prev_right_pos is not None and start_teleop:
            
            # Parse the message
            left_pose, right_pose = parse_pose_message(message)


            if time.time() - last_ts >= CONTROL_TIME_PERIOD:
            
                rel_left_pose[0] += (left_pose[0] - prev_left_pos[0])*SCALE_FACTOR
                
                # magnitude = np.linalg.norm(rel_left_pose)
                print(np.linalg.norm(rel_left_pose[:3]))
                if (np.linalg.norm(rel_left_pose[:3]) > 1):
                    print("Sending to robot")
                    # TODO: FIXME; HACK. quaternion so, 
                    queue.put(CartesianMoveMessage(target=rel_left_pose[:6], wait=False, relative=True)) #For now, only consider the position
                    # Reset the relative pose
                    rel_left_pose = [0. for  _ in range(7)]
                    # Update the previous pose
                    prev_left_pos = left_pose
                    prev_right_pos = right_pose

            socket.send_string("OK: {}".format(time.time()))


import time
from xarm import XArmAPI
import multiprocessing as mp
from bimanual.hardware.robot.xarmrobot import register_ctx, CustomManager, CartesianMoveMessage
from ctypes import c_double

def move_robot(queue, last_sent_msg_ts, last_sent_msg, control_timeperiod):
    # Initialize xArm API
    arm = XArmAPI("192.168.86.216")

    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=False)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.move_gohome(wait=True)
    time.sleep(0.1)
    
    status, pose = arm.get_position_aa()
    assert status==0, "Failed to get robot position"
    print("Robot position: {}".format(pose))
    
    # exit()
    arm.set_mode(1)
    arm.set_state(0)

    time.sleep(0.1)

    while True: # Use a lock instead
        if time.time() - last_sent_msg_ts.value > control_timeperiod:
            print(queue.empty())

            if not queue.empty():
                print("Sending message to robot: {}".format(queue.qsize()))
                move_msg = queue.get()
                print(move_msg)
                if move_msg.is_terminal():
                    break
                #TODO : Move to target check for tolerance in step_size. - No; move this to the gym environment.
                # print(vars(self))
                print(move_msg.target)
                print(vars(move_msg))

                # y = bot.get_position()
                for i in range(len(move_msg.target)):
                    pose[i]+=move_msg.target[i]
                print("Sending robot to: {}".format(pose))
                # x = arm.set_servo_cartesian_aa(pose, wait=False, relative=False, mvacc=200, speed=100)

                if pose[0]>406 or pose[0]<206 or pose[1]>200 or pose[1]<-200:
                    print("Robot out of bounds")
                    if pose[0]>406:
                        pose[0]=406
                    elif pose[0] < 206:
                        pose[0]=206
                    if pose[1]>200:
                        pose[1]=200
                    elif pose[1] < -200:
                        pose[1]=-200
                    
                    continue

                x = arm.set_servo_cartesian_aa(move_msg.target, wait=False, relative=True, mvacc=200, speed=100)
                print("Robot set_servo_cartesian returns: {}".format(x))
                if x!=0:
                    print("Failed to set robot position")
                    break
                # x = arm.set_position(*move_msg.target, relative=True, wait=False)
                # print("Robot set_position returns: {}".format(x))

                last_sent_msg_ts = c_double(time.time())
                last_sent_msg = move_msg
                print("last_sent_msg: {}".format(last_sent_msg.target))


            else:
                time.sleep(0.001)
        
        
        
if __name__ == "__main__":

    control_frequency = 90# TODO; measure control frequency for each box. COmmands should be sent at 30-250. If <30 - choppy; if>250 drop frames. Deatils in manual:
    control_timeperiod = 1./control_frequency

    register_ctx()
    ctx_manager = CustomManager(ctx=mp.get_context())
    ctx_manager.start()


    last_sent_msg_ts = mp.Value("d", time.time(), lock=False) # Single process access; no need for lock; Makes process faster.
    # Last message sent to the message queue by the calling process
    last_queued_msg = ctx_manager.CartesianMoveMessage([0, 0, 0, 0, 0, 0], relative =True) # TODO : Implement this as mp object; use BaseManager;


    message_queue = mp.Queue()
    moving_process = mp.Process(target=move_robot, args=(message_queue, last_sent_msg_ts, last_queued_msg, control_timeperiod), name = "move_proc")
    start_server_process = mp.Process(target= start_server,args =(message_queue,), name="server_thread")

    moving_process.start()
    start_server_process.start()
