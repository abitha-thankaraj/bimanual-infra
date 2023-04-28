# x,y,z to z,x,y
import numpy as np

FLIP_MATRIX = np.array(
    [
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0]
    ]
)


# TODO: Move this to init file
x_max = 406
x_min = 206
y_max = 200
y_min = -200
z_max = 420
z_min = 280

SCALE_FACTOR = 1000
CONTROL_FREQ = 90.
CONTROL_TIME_PERIOD = 1/CONTROL_FREQ
