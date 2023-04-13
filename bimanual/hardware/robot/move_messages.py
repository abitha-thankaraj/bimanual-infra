from bimanual.hardware.robot.xarmrobot import CartesianMoveMessage


def move_x(x=10):
    return CartesianMoveMessage(
        target = [x, 0, 0, 0, 0, 0],
        relative=True
    )

def move_y(y=10):
    return CartesianMoveMessage(
        target = [0, y, 0, 0, 0, 0],
        relative=True
    )

def move_z(z=10):
    return CartesianMoveMessage(
        target = [0, 0, z, 0, 0, 0],
        relative=True
    )

