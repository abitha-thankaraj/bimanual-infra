import zmq

def server(port):
    context = zmq.Context()
    socket = context.socket(zmq.ROUTER)
    socket.bind("tcp://*:{}".format(port))
    return socket

def client(client_id, port):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.identity = client_id.encode()
    socket.connect("tcp://localhost:{}".format(port))
    return socket


def start_server(port):
    socket = server(port)
    print("Server started.")
    k =0

    while True:
        k+=1
        print("Waiting for requests...")
        # Wait for a request from a client
        received= socket.recv_multipart()
        client_id, request = received[0], received[2]
        print(f"Received request: {request.decode()}")

        # Process the request (you can modify this part according to your application's logic)
        response = f"Hello, client {client_id.decode()}!"
        print(f"Received request: {request.decode()}")
        print(f"Sending response: {response}")
        if k == 4:
            exit()
        # Send the response back to the client
        socket.send_multipart([client_id, b'', response.encode()])


def start_client(client_id, port):

    socket = client(client_id, port)
    print(f"Client {client_id} started.")

    while True:
        # Send a request to the server
        request = "Enter a message to send to the server (or 'exit' to quit): "
        if request == "exit":
            break

        print("Hello")
        socket.send(request.encode())

        # Wait for the reply from the server
        response = socket.recv()
        print(f"Received response: {response.decode()}")


if __name__ == "__main__":
    # Start the server as a background process
    import multiprocessing as mp
    socket = server(SERVER_PORT)
    server_process = mp.Process(target=start_server, args=(5556,))

    # Start the clients as background processes

    client_processes = [mp.Process(target=start_client, args=("client_{}".format(i), 5556)) for i in range(2)]

    # Start the processes
    server_process.start()
    for client_process in client_processes:
        client_process.start()

    # Wait for the processes to finish
    server_process.join()
    for client_process in client_processes:
        client_process.join()