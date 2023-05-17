import time
import numpy as np
from gym import Env

from bimanual.hardware.robot import Robot
from bimanual.servers import RIGHT_ARM_IP, LEFT_ARM_IP


class BimanualEnv(Env):
    def __init__(self):
        # Start robots
        self.left_arm = Robot(LEFT_ARM_IP)
        self.right_arm = Robot(RIGHT_ARM_IP)
        # Start cameras

    def step(self, action):
        # Split action into left and right arm actions
        qpos_l = action[:8]
        gripper_l = action[8]

        qpos_r = action[8:]
        gripper_r = action[15]

        # TODO: Wait for both arms to finish moving? This should be determined by opening and closignof grippers.
        # TODO: Asyncio foo to wait until both arms are done moving.
        # Might cause object to fall.
        self.left_arm.move(qpos_l, wait=True)
        self.right_arm.move(qpos_r, wait=True)
        # TODO: Add gripper logic
        # Send actions to queues? if you want them to execute in parallel.

        return None, None, None, None

    def reset(self):
        # Move both arms to home position

        # Retun observation
        return None

    def render(self):
        # Not sure if we need this. Render - Pull up camera on screen.
        pass

    def close(self):
        # Cleanup; Close all threads/processes
        pass
