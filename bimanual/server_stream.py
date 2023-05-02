import multiprocessing as mp
from bimanual.servers.state_server import start_server_stream

if __name__ == "__main__":
    queue = mp.Queue()

    try:
        # Start the server
        server_process = mp.Process(target = start_server_stream, args=(queue,), name = "server_stream_proc")
        server_process.start()


        while True:
            print("Waiting for message...")
            controller_state = queue.get()
            
                        
            from IPython import embed; embed()
    
    # keyboard interrupt exception
    except KeyboardInterrupt:
        server_process.join()

        print("Exiting...")
        exit()
