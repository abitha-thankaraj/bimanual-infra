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


