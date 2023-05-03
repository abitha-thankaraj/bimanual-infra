import zmq
import time
from bimanual.utils.debug_utils import DebugTimer


def start_server():
    # Set up the ZeroMQ context
    context = zmq.Context()

    # Create a REP (reply) socket
    socket = context.socket(zmq.REP)
    # Bind the socket to a specific address and port
    socket.bind("tcp://*:5555")

    print("Starting server...")

    last_ts = time.time()
    while True:
        # Wait for a message from the client
        message = socket.recv_string()
        # print(message)
        socket.send_string("OK: Received message")

        current_ts = time.time()

        print("Time since last message: ", current_ts - last_ts)
        print("Frequency: ", 1.0 / (current_ts - last_ts))
        last_ts = current_ts


if __name__ == "__main__":
    start_server()
