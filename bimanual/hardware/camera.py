import numpy as np
import subprocess as sp
import pyrealsense2 as rs
import multiprocessing as mp
from bimanual.servers import DATA_DIR

class RealSenseCameraProcess(mp.Process):
    def __init__(self, serial_number, output_file, exit_event: mp.Event):
        super(RealSenseCameraProcess, self).__init__()
        self.serial_number = serial_number
        self.output_file = output_file
        self.pipeline = None
        self.align = None
        self.ffmpeg_process = None
        self.exit_event = exit_event

    def run(self):
        # Create a pipeline and alignment object
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)

        # Create a configuration object and enable streams
        config = rs.config()
        config.enable_device(self.serial_number)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start the pipeline
        self.pipeline.start(config)
        for _ in range(100):
            self.pipeline.wait_for_frames()

        # Configure ffmpeg command for writing video
        # TODO: Fix ffmpeg command to write video with correct timestamp?
        ffmpeg_command = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-s', '640x480',
            '-i', '-',
            '-r', '30',
            '-c:v', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-crf', '18',
            self.output_file
        ]

        # Start ffmpeg process
        self.ffmpeg_process = sp.Popen(ffmpeg_command, stdin=sp.PIPE)

        try:
            while not self.exit_event.is_set():
                # Wait for a coherent pair of frames (color and depth)
                frames = self.pipeline.wait_for_frames()

                # Align depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get aligned color and depth frames
                aligned_color_frame = aligned_frames.get_color_frame()

                #TODO: Record depth frames if needed.

                # Perform further processing or analysis on the aligned frames
                if aligned_color_frame:
                    # Access the frame data using aligned_color_frame.get_data()
                    frame_data = aligned_color_frame.get_data()

                    # Write frame data to ffmpeg stdin pipe
                    self.ffmpeg_process.stdin.write(frame_data)

        finally:
            # Close ffmpeg stdin pipe and wait for the process to finish
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.wait()
            self.pipeline.stop()

    def terminate(self):
        super(RealSenseCameraProcess, self).terminate()
        if self.ffmpeg_process:
            self.ffmpeg_process.terminate()








# class Camera:
#     def __init__(self, width=640, height=480, connect=False, camera_id=None):
#         self._width = width
#         self._height = height
#         self._camera_id = camera_id

#         if connect:
#             self._connect_cam()

#     def __enter__(self):
#         self._connect_cam()
#         return self

#     def __exit__(self, exc_type, exc_val, exc_tb):
#         self.release()

#     def _connect_cam(self):
#         self.pipeline = rs.pipeline()

#         # Configure streams
#         config = rs.config()

#         if self._camera_id is not None:
#             config.enable_device(self._camera_id)

#         config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, 30)
#         config.enable_stream(rs.stream.color, self._width, self._height, rs.format.rgb8, 30)

#         # Start streaming and align frames
#         self.profile = self.pipeline.start(config)
#         align_to = rs.stream.color
#         self.align = rs.align(align_to)

#         for _ in range(100):  # Skip th first few frames - dark/blurry frames
#             frames = self.pipeline.wait_for_frames()

#     def get_frame(self):

#         frames = self.pipeline.wait_for_frames()
#         aligned_frames = self.align.process(frames)

#         aligned_depth_frame = aligned_frames.get_depth_frame()
#         aligned_color_frame = aligned_frames.get_color_frame()
#         if not aligned_depth_frame or not aligned_color_frame:
#             raise Exception("ERROR: no new images receieved")

#         return (aligned_color_frame.get_timestamp() / 1000, np.asarray(aligned_color_frame.get_data()), np.asarray(aligned_depth_frame.get_data()))

#     def __write_frames__(self):
#         ts, rgb, depth = self.get_frame()
#         self.process.stdin.write(rgb.tobytes())

#     def release(self):
#         self.pipeline.stop()


# def start_video_recording(cam: Camera,
#                           exit_event: mp.Event = None,
#                           traj_id: str = None,
#                           frame_rate: int = 30):

#     output_filename = '{}/{}/video.mp4'.format(DATA_DIR, traj_id)

#     ts, rgb, depth = cam.get_frame()
#     height, width, channel = rgb.shape  # 480, 600, 3


#     print("Initializing ffmpeg...")
#     command = [
#         'ffmpeg',
#         '-y',  # Overwrite output files if they already exist
#         '-f', 'rawvideo',
#         '-vcodec', 'rawvideo',
#         '-s', f'{width}x{height}',  # Width x Height of the input frames
#         '-pix_fmt', 'rgb24',  # Input pixel format (RGB)
#         '-r', f'{frame_rate}',  # Frames per second
#         '-i', '-',  # Read input from pipe
#         '-c:v', 'libx264',  # Output video codec
#         '-pix_fmt', 'yuv420p',  # Output pixel format (YUV420p)
#         output_filename  # Output filename
#     ]
#     cam.process = sp.Popen(command, stdin=sp.PIPE)
#     while exit_event is None or not exit_event.is_set():
#         print("Writing frame...")
#         cam.__write_frames__()

#     print("Terminate Recording...")
#     cam.process.stdin.close()
#     cam.process.wait()
#     print("Recording Terminated")
#     return
