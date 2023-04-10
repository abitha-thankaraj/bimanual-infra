from enum import Enum
from xarm.wrapper import XArmAPI

class RobotControlMode(Enum):
    JOINT_POSITION = 0
    CARTESIAN_POSITION = 0
    
class Robot(XArmAPI):
    def __init__(self, ip, port=0, do_not_open=False, robot_control_mode=RobotControlMode.CARTESIAN_POSITION, is_radian=False):
        super(Robot, self).__init__(ip, port, do_not_open, is_radian)
        self._control_mode = robot_control_mode

    def reset(self, home = False):
        self.clean_error()
        self.clean_warn()
        self.motion_enable(enable=True)
        self.set_mode(self._control_mode)
        self.set_state(state=0)
    
        if home:
            self.home_robot()
    
    def home_robot(self):
        pass

    def move(self, target):
        pass