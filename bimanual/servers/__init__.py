import numpy as np

# Controller to robot rotation.
FLIP_MATRIX = np.array(
    [
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0]
    ]
)


# Robot workspace position limits.
x_min, x_max = 206, 506
y_min, y_max = -200, 200
z_min, z_max = 280, 420

ROBOT_WORKSPACE = np.array([[x_min, x_max, y_min], [y_max, z_min, z_max]])

ROBOT_SERVO_MODE_STEP_LIMITS = np.array([-5, 5])
# Robot home pose.
ROBOT_HOME_POSE_AA = [206.0, -0.0, 475, np.pi, -0.0, 0.0]

# Convrsion controller <-> robot position coordinates. m <-> mm
SCALE_FACTOR = 1000.0

#TODO: Check if this is still needed in vr controller code.
CONTROL_FREQ = 90. # Commands should be sent at 30-250. If <30 - choppy; if>250 drop frames. Details in manual:
CONTROL_TIME_PERIOD = 1/CONTROL_FREQ

RIGHT_ARM_IP = "192.168.86.230"
LEFT_ARM_IP = "192.168.86.216"