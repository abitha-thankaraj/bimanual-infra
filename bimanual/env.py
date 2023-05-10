import time
import numpy as np
from gym import Env

from bimanual.servers import RIGHT_ARM_IP, LEFT_ARM_IP


class BimanualEnv(Env):
    def __init__(self):
        pass

    def step(self, action):
        # Split action into left and right arm actions
        qpos_r = action[:7]
        qpos_l = action[7:]

        # Send actions to queues
        pass

    def reset(self):
        # Move both arms to home position
        pass

    def render(self):
        # Not sure if we need this
        pass

    def close(self):
        # Cleanup; Close all threads/processes
        pass
