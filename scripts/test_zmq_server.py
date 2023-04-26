import zmq
import time
import multiprocessing as mp
import numpy as np
from xarm import XArmAPI
from ctypes import c_double
import pandas as pd
from bimanual.hardware.robot.xarmrobot import  register_ctx, CustomManager, CartesianMoveMessage
from bimanual.utils.debug_utils import DebugTimer 




def start_server():
    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    
    socket = context.socket(zmq.REP)
    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    print("Starting server...")


    while True:
        # Wait for a message from the client
        message = socket.recv_string()
        print(message)
        socket.send_string("OK: Received message")
        



SCALE_FACTOR = 1000
CONTROL_FREQ = 90.
CONTROL_TIME_PERIOD = 1/CONTROL_FREQ


def parse_pose_message(message):
    left_pose, right_pose = message.split('|')
    left_pose = [round(float(x),3) for x in left_pose.split(',')]
    right_pose = [round(float(x),3) for x in right_pose.split(',')]
    return left_pose, right_pose


if __name__ == "__main__":
    start_server()