import zmq
import time


def start_subscriber():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    # Set the ZMQ_RCVHWM option
    socket.setsockopt(zmq.RCVHWM, 1)

    # Connect to the publisher
    socket.connect("tcp://10.19.238.56:5555")

    # Subscribe to messages
    socket.subscribe(b"oculus_controller")

    i = 0
    last_ts = time.time()

    # Receive messages
    while True:
        try:
            print(i)
            i += 1
            msg = socket.recv_string()
            print(msg)
            current_ts = time.time()
            


            print("Time since last message: ", current_ts - last_ts)
            print("Frequency: ", 1.0 / (current_ts - last_ts))
            last_ts = current_ts

        except zmq.error.ContextTerminated:
            break


if __name__ == "__main__":
    start_subscriber()
