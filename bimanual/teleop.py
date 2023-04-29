import multiprocessing as mp

from bimanual.hardware.robot import move_robot
from bimanual.servers.state_server import start_server
from bimanual.servers import RIGHT_ARM_IP, LEFT_ARM_IP


if __name__ == "__main__":

    right_message_queue = mp.Queue()
    left_message_queue = mp.Queue()

    try:
        right_moving_process = mp.Process(target = move_robot, args=(right_message_queue, RIGHT_ARM_IP), name = "move_robot_right_proc")
        # left_moving_process = mp.Process(target = move_robot, args=(left_message_queue, LEFT_ARM_IP), name = "move_robot_left_proc")
        start_server_process = mp.Process(target= start_server, args =(left_message_queue, right_message_queue,), name="server_proc")

        right_moving_process.start()
        # left_moving_process.start()
        start_server_process.start()
    
    # keyboard interrupt exception
    except KeyboardInterrupt:
        right_moving_process.join()
        # left_moving_process.join()
        start_server_process.join()

        print("Exiting...")
        exit()

    except Exception as e:
        #TODO: Bubble up exitcode/exceptions from move_robot, start_server
        print("Error: {}".format(e))
        exit()
