import pyrealsense2 as rs
import datetime

# Connect to the RealSense camera
pipeline = rs.pipeline()
config = rs.config()
pipeline.start(config)

# Get the current system time
system_time = datetime.datetime.now()

# Get the current camera timestamp
frames = pipeline.wait_for_frames()
timestamp_camera = frames.get_timestamp()

# Calculate the time difference between camera timestamp and system time
time_difference = timestamp_camera - system_time.timestamp()

# Set the camera's time
device = pipeline.get_active_profile().get_device()
device.hardware_reset()
device.set_time_base(time_difference)

# Stop the pipeline
pipeline.stop()
