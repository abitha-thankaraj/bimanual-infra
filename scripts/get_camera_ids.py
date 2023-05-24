import pyrealsense2 as rs
# from logging import logger

# Create a context object for managing connected devices
ctx = rs.context()

# Get a list of all connected devices
devices = ctx.query_devices()

# Print the serial numbers of all connected devices
for device in devices:
    print("Device Serial Number:", device.get_info(rs.camera_info.serial_number))

    # logger.log("Hello")
# # Select a specific camera by serial number
# selected_serial_number = "<serial_number_of_desired_camera>"
# selected_device = None
# from IPython import embed; embed()
# for device in devices:
#     if device.get_info(rs.camera_info.serial_number) == selected_serial_number:
#         selected_device = device
#         break

# # Connect to the selected camera
# if selected_device:
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_device(selected_device.get_info(rs.camera_info.serial_number))
#     pipeline.start(config)
#     print("Connected to camera:", selected_device.get_info(rs.camera_info.serial_number))
# else:
#     print("Desired camera not found.")
