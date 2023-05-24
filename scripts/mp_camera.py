import numpy as np
import multiprocessing
import pyrealsense2 as rs


def camera_worker(output_pipe):
    # Configure the RealSense camera pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # Start the pipeline
    pipeline.start(config)
    for _ in range(100):
        pipeline.wait_for_frames()

    try:
        while True:
            # Wait for the next frame from the camera
            frames = pipeline.wait_for_frames()

            # Retrieve the color frame
            color_frame = frames.get_color_frame()

            # Convert the color frame to a numpy array
            color_array = np.asanyarray(color_frame.get_data())

            # Send the numpy array to the parent process via the output pipe
            output_pipe.send(color_array)
    finally:
        # Stop the pipeline and release resources
        pipeline.stop()


def main():
    # Create a pipe for interprocess communication
    parent_conn, child_conn = multiprocessing.Pipe()

    # Create a child process for camera capture
    camera_process = multiprocessing.Process(target=camera_worker, args=(child_conn,))
    camera_process.start()

    color_frames = []
    try:
        for i in range(10):
            # Receive the color frame from the child process
            color_frame = parent_conn.recv()
            color_frames.append(color_frame)

            # Process the color frame or perform desired actions
            # ...

    finally:
        # Terminate the camera process and close the pipe
        camera_process.terminate()
        parent_conn.close()
        child_conn.close()
        return color_frames


if __name__ == '__main__':
    color_frames = main()
    from IPython import embed
    embed()
