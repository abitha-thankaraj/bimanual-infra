import time
from xarm.wrapper import XArmAPI
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
    while True:
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
                x = arm.set_position(*move_msg.target, relative=True, wait=False)
                print("Robot set_position returns: {}".format(x))

                last_sent_msg_ts = c_double(time.time())
                last_sent_msg = move_msg
                print("last_sent_msg: {}".format(last_sent_msg.target))


            else:
                time.sleep(0.001)
        
        
        if not queue.empty():
            print("Sending message to robot: {}".format(queue.qsize()))
            move_msg = queue.get()
            if move_msg.is_terminal():
                break
            #TODO : Move to target check for tolerance in step_size. - No; move this to the gym environment.
            # print(vars(self))
            print(move_msg.target)
            print(vars(move_msg))

            # y = arm.get_position()
            # print("Robot position: {}".format(y))

            last_sent_msg_ts = c_double(time.time())
            # print(last_received_msg.target)
            last_received_msg = move_msg
            x = arm.set_position(*move_msg.target, relative=True, wait=True)
            arm.set_servo_cartesian
            print("Robot command returned: {}".format(x))
        else:
            time.sleep(1)

if __name__ == "__main__":

    control_frequency = 100# TODO; measure control frequency for each box. COmmands should be sent at 30-250. If <30 - choppy; if>250 drop frames. Deatils in manual:
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

    dummy_msg = CartesianMoveMessage([5, 0, 0, 0, 0, 0], relative =True, wait=False)
    for _ in range(10):
        message_queue.put(dummy_msg)

    from IPython import embed; embed()
    moving_process.join()
    exit()