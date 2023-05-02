import time
from xarm import XArmAPI

def cmd_callback(data):
    print(f"Command response: {data}")

# Replace this with your xArm's IP address
xarm_ip = '192.168.86.216'

# Initialize xArm API
arm = XArmAPI(xarm_ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)

# Send a series of commands and measure the time
num_commands = 100
start_time = time.time()

for i in range(num_commands):
    # arm.move_gohome(wait=False)
    arm.get_position()
    # arm.send_cmd_async(f'set_state,0', cmd_callback) # TODO: Fix this. It's garbage.

end_time = time.time()
elapsed_time = end_time - start_time

control_frequency = num_commands / elapsed_time
print(f"Control frequency: {control_frequency} commands/s")
from IPython import embed; embed()
# Clean up
arm.disconnect()
