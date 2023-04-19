import zmq
import time
import multiprocessing as mp

from bimanual.hardware.robot.xarmrobot import CartesianMoveMessage

SCALE_FACTOR = 10

def start_server(queue: mp.Queue):
    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    socket = context.socket(zmq.REP)
    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    start_teleop = False
    prev_left_pos, prev_right_pos = None, None
    init_pos = False

    print("Starting server...")

    while True:
        # Wait for a message from the client
        message = socket.recv_string()
        print("Received request: {}, at time:{}".format(message, time.time()))

        if message.startswith("start"):
            start_teleop = True
            socket.send_string("OK: Starting teleop at time:{}".format(time.time()))
            init_pos = True
            continue    
        
        if init_pos:
            left_pose, right_pose = message.split('|')
            left_pose = [float(x) for x in left_pose.split(',')]
            right_pose = [float(x) for x in right_pose.split(',')]
            prev_left_pos = left_pose
            prev_right_pos = right_pose
            init_pos = False
            socket.send_string("OK: Received init pos")
            continue
        
        if prev_left_pos is not None and prev_right_pos is not None and start_teleop:
            # TODO: Parse the message; make function
            
            # Parse the message
            left_pose, right_pose = message.split('|')
            left_pose = [float(x) for x in left_pose.split(',')]
            right_pose = [float(x) for x in right_pose.split(',')]

            # # Relative pose : For now, on;y consider the position
            rel_left_pose = [(left_pose[i] - prev_left_pos[i])*SCALE_FACTOR for i in range(3)] # Scale by 10?
            rel_right_pose = [right_pose[i] - prev_right_pos[i] for i in range(3)]

            # TODO: Add to queue to send the relative pose to the robot
            rel_left_pose.extend([0. ,0., 0.])
            queue.put(CartesianMoveMessage(target=rel_left_pose, wait=False, relative=True)) #For now, only consider the position

            # Update the previous pose
            prev_left_pos = left_pose
            prev_right_pos = right_pose

            # Print the pose
            print("Left:", left_pose)
            print("Right:", right_pose)


            # Send an acknowledgment back to the client
            socket.send_string("OK: {}".format(time.time()))

if __name__ == "__main__":
    message_queue = mp.Queue()
    start_server(message_queue)