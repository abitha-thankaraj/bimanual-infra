import zmq

# Set up the ZeroMQ context
context = zmq.Context()

# Create a REQ (request) socket
socket = context.socket(zmq.REQ)
# Connect the socket to the server address
socket.connect("tcp://10.19.216.156:5555")

# Send position and rotation data to the server
position = [1.0, 2.0, 3.0]
rotation = [45.0, 30.0, 60.0]

message = f"{','.join(map(str, position))}|{','.join(map(str, rotation))}"
socket.send_string(message)

# Receive the acknowledgment from the server
reply = socket.recv_string()
print(f"Received reply: {reply}")