import time
from xarm import XArmAPI
import multiprocessing as mp
from bimanual.hardware.scratch_robot.xarmrobot import register_ctx, CustomManager, CartesianMoveMessage
from ctypes import c_double
from keyboard_ctrl import keyboard_control

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
               
            if not queue.empty():
                print("Sending message to robot: {}".format(queue.qsize()))
                move_msg = queue.get()
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

    moving_process.start()
    #TODO: Lower control frequency to stop messages from dropping; I suspect that this is what causes the drift

    # dummy_msg = CartesianMoveMessage([2, 0, 0, 0, 0, 0], relative =True, wait=False)


    # for i in range(200):
    #     for _ in range(60):
    #         message_queue.put(CartesianMoveMessage([2, 0, 0, 0, 0, 0], relative =True, wait=False))
    #     for _ in range(60):
    #         message_queue.put(CartesianMoveMessage([0, -2, 0, 0, 0, 0], relative =True, wait=False))
    #     for _ in range(60):
    #         message_queue.put(CartesianMoveMessage([-2, 0, 0, 0, 0, 0], relative =True, wait=False))
    #     for _ in range(60):
    #         message_queue.put(CartesianMoveMessage([0, 2, 0, 0, 0, 0], relative =True, wait=False))
    


    keyboard_control(message_queue)
    time.sleep(5)
    from IPython import embed; embed()
    moving_process.join()
    exit()