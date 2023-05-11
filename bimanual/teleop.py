import numpy as np
import multiprocessing as mp

from bimanual.hardware.robot import move_robot
from bimanual.servers.state_subscriber import start_subscriber
from bimanual.servers import RIGHT_ARM_IP, LEFT_ARM_IP


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)

    right_message_queue = mp.Queue()
    left_message_queue = mp.Queue()
    exit_event = mp.Event()

    try:
        # Right arm
        right_moving_process = mp.Process(target=move_robot,
                                          args=(right_message_queue,
                                                RIGHT_ARM_IP,
                                                exit_event),
                                          name="move_robot_right_proc")
        # Left arm
        left_moving_process = mp.Process(target=move_robot,
                                         args=(left_message_queue,
                                               LEFT_ARM_IP,
                                               exit_event),
                                         name="move_robot_left_proc")
        # Server to receive state from Oculus
        start_subscriber_process = mp.Process(target=start_subscriber,
                                              args=(left_message_queue,
                                                    right_message_queue,
                                                    exit_event),
                                              name="subscriber_proc")

        # TODO: Add a process to record camera data

        processes = [right_moving_process, left_moving_process, start_subscriber_process]

        for process in processes:
            process.start()

        while True:
            if exit_event.is_set():
                break  # Takes you to the finally block

    # keyboard interrupt exception;
    # TODO: Use a button on the controller to exit; Maybe some lock shared across processes?
    except KeyboardInterrupt:

        print("Exiting...")

    except Exception as e:
        # TODO: Bubble up exitcode/exceptions from move_robot, start_server
        print("Error: {}".format(e))
        exit()

    finally:
        # Cleanup

        # TODO: Move to a function. if process.is_alive() then process.terminate()

        for process in processes:
            if process.is_alive():
                process.join()
                process.terminate()

        print("Exiting...")
        exit()
