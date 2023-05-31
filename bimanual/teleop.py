import os
import time
import datetime
import numpy as np
import multiprocessing as mp

from bimanual.hardware.robot import move_robot
from bimanual.hardware.camera import RealSenseCameraProcess
from bimanual.servers.state_subscriber import start_subscriber
from bimanual.servers import RIGHT_ARM_IP, LEFT_ARM_IP, DATA_DIR, CAMERA_IDS


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)

    right_message_queue = mp.Queue()
    left_message_queue = mp.Queue()
    exit_event = mp.Event()

    try:
        # Create a directory to save data
        traj_id = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        save_dir = os.path.join(DATA_DIR, traj_id)
        os.makedirs(save_dir, exist_ok=True)

        # Right arm
        right_moving_process = mp.Process(target=move_robot,
                                          args=(right_message_queue,
                                                RIGHT_ARM_IP,
                                                exit_event,
                                                traj_id),
                                          name="move_robot_right_proc")
        # Left arm
        left_moving_process = mp.Process(target=move_robot,
                                         args=(left_message_queue,
                                               LEFT_ARM_IP,
                                               exit_event,
                                               traj_id),
                                         name="move_robot_left_proc")
        # Subscriber to consume controller state from Oculus
        start_subscriber_process = mp.Process(target=start_subscriber,
                                              args=(left_message_queue,
                                                    right_message_queue,
                                                    exit_event),
                                              name="subscriber_proc")
        # Simulation process to simulate the robot
        start_simulation_process = mp.Process(target=start_simulation_with_zmq,
                                              args=(exit_event,),
                                              name="simulation_proc")
        # Camera processes for all realsense cameras attached.
        camera_processes = [RealSenseCameraProcess(serial_number="{}".format(camera_id), 
                                                   output_file= "video_{}_rgb.mp4".format(camera_id),
                                                   exit_event = exit_event) for camera_id in CAMERA_IDS]

        processes = [
            right_moving_process, # Right robot arm
            left_moving_process, # Left robot arm
            start_subscriber_process # Subscriber
            start_simulation_process # Simulation
        ]
        
        # Add camera processes to the list of processes
        processes.extend(camera_processes)

        for process in processes:
            process.start()

        while True:
            # Keep main thread running until exit event is set by the subscriber.
            if exit_event.is_set():
                time.sleep(10) 
                # Wait until all the writes are complete. 
                # Correct way to do this -> Use counting lock for all camera procesess to complete writing.
                break  # Takes you to the finally block

    except Exception as e:
        # TODO: Bubble up exitcode/exceptions from move_robot, start_server
        print("Error: {}".format(e))
        exit()

    finally:
        # Cleanup
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join()

        print("Exiting...")
        exit()
