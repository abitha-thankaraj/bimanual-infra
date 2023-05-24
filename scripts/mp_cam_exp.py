import pyrealsense2 as rs
from multiprocessing import Process
import subprocess


class RealSenseCameraProcess(Process):
    def __init__(self, serial_number, rgb_output_file, depth_output_file):
        super(RealSenseCameraProcess, self).__init__()
        self.serial_number = serial_number
        self.rgb_output_file = rgb_output_file
        self.depth_output_file = depth_output_file
        self.pipeline = None
        self.align = None
        self.rgb_ffmpeg_process = None
        self.depth_ffmpeg_process = None

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

        # Configure ffmpeg command for writing RGB frames to video
        rgb_ffmpeg_command = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-s', '640x480',
            '-i', '-',
            '-c:v', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-preset', 'ultrafast',
            '-crf', '18',
            self.rgb_output_file
        ]

        # Start ffmpeg process for RGB frames
        self.rgb_ffmpeg_process = subprocess.Popen(rgb_ffmpeg_command, stdin=subprocess.PIPE)

        # Configure ffmpeg command for writing depth frames to video
        depth_ffmpeg_command = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-s', '640x480',
            '-i', '-',
            '-c:v', 'libx264',
            '-pix_fmt', 'yuv420p',
            '-preset', 'ultrafast',
            '-crf', '18',
            self.depth_output_file
        ]

        # Start ffmpeg process for depth frames
        self.depth_ffmpeg_process = subprocess.Popen(depth_ffmpeg_command, stdin=subprocess.PIPE)

        try:
            while True:
                # Wait for a coherent pair of frames (color and depth)
                frames = self.pipeline.wait_for_frames()

                # Align depth frame to color frame
                aligned_frames = self.align.process(frames)

                # Get aligned color and depth frames
                aligned_color_frame = aligned_frames.get_color_frame()
                aligned_depth_frame = aligned_frames.get_depth_frame()

                # Perform further processing or analysis on the aligned frames
                if aligned_color_frame and aligned_depth_frame:
                    # Access the frame data using aligned_color_frame.get_data() and aligned_depth_frame.get_data()
                    color_data = aligned_color_frame.get_data()
                    depth_data = aligned_depth_frame.get_data()

                    # Write color frame data to ffmpeg stdin pipe
                    self.rgb_ffmpeg_process.stdin.write(color_data)

                    # Write depth frame data to ffmpeg stdin pipe
                    self.depth_ffmpeg_process.stdin.write(depth_data)

        finally:
            # Close ffmpeg stdin pipes and wait for the processes to finish
            self.rgb_ffmpeg_process.stdin.close()
            self.rgb_ffmpeg_process.wait()
            self.depth_ffmpeg_process.stdin.close()
            self.depth_ffmpeg_process.wait()


# class RealSenseCameraProcess(Process):
#     def __init__(self, serial_number, output_file):
#         super(RealSenseCameraProcess, self).__init__()
#         self.serial_number = serial_number
#         self.output_file = output_file
#         self.pipeline = None
#         self.align = None
#         self.ffmpeg_process = None

#     def run(self):
#         # Create a pipeline and alignment object
#         self.pipeline = rs.pipeline()
#         self.align = rs.align(rs.stream.color)

#         # Create a configuration object and enable streams
#         config = rs.config()
#         config.enable_device(self.serial_number)
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

#         # Start the pipeline
#         self.pipeline.start(config)
#         for _ in range(100):
#             self.pipeline.wait_for_frames()

#         # Configure ffmpeg command for writing video
#         ffmpeg_command = [
#             'ffmpeg',
#             '-y',
#             '-f', 'rawvideo',
#             '-pix_fmt', 'rgb24',
#             '-s', '640x480',
#             '-i', '-',
#             '-c:v', 'libx264',
#             '-pix_fmt', 'yuv420p',
#             '-preset', 'ultrafast',
#             '-crf', '18',
#             self.output_file
#         ]

#         # Start ffmpeg process
#         self.ffmpeg_process = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)

#         try:
#             while True:e
#                 # Wait for a coherent pair of frames (color and depth)
#                 frames = self.pipeline.wait_for_frames()

#                 # Align depth frame to color frame
#                 aligned_frames = self.align.process(frames)

#                 # Get aligned color and depth frames
#                 aligned_color_frame = aligned_frames.get_color_frame()

#                 # Perform further processing or analysis on the aligned frames
#                 if aligned_color_frame:
#                     # Access the frame data using aligned_color_frame.get_data()
#                     frame_data = aligned_color_frame.get_data()

#                     # Write frame data to ffmpeg stdin pipe
#                     self.ffmpeg_process.stdin.write(frame_data)

#         finally:
#             # Close ffmpeg stdin pipe and wait for the process to finish
#             self.ffmpeg_process.stdin.close()
#             self.ffmpeg_process.wait()

    def terminate(self):
        super(RealSenseCameraProcess, self).terminate()
        if self.rgb_ffmpeg_process:
            self.rgb_ffmpeg_process.terminate()
        if self.depth_ffmpeg_process:
            self.depth_ffmpeg_process.terminate()
        # if self.ffmpeg_process:
        #     self.ffmpeg_process.terminate()


# Usage example
if __name__ == '__main__':
    camera_ids = ["023322062082", "239122072217"]
    # Create camera processes for each desired camera
    camera_processes = [RealSenseCameraProcess("{}".format(camera_id), "output_{}_rgb.mp4".format(camera_id), "output_{}_dep.mp4".format(camera_id)) for camera_id in camera_ids]

    # Start camera processes
    for process in camera_processes:
        process.start()
    print("Started camera processes")
    print("Sleeping for 5 seconds")
    # Wait for some time (e.g., 5 seconds)
    # Replace this with your desired logic or application flow
    import time
    time.sleep(15)
    print("Done sleeping")
    # Terminate camera processes
    for process in camera_processes:
        process.terminate()

    # Join camera processes
    for process in camera_processes:
        process.join()

    from IPython import embed
    embed()
