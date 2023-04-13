import gym
from bimanual.hardware.robot.xarmrobot import XArmRobot
from bimanual.hardware.robot.mp_utils import register_ctx
import zerorpc

class BimanualEnv(gym.Env):
    def __init__(self):

        register_ctx()
        
        # TODO : Use zerorpc to communicate with robot. skeleton code below.
        # Add read from file 

        # Run robot servers in separate processes
        robot_server_left = zerorpc.Server(XArmRobot(ip="192.168.86.230"))
        robot_server_left.bind("tcp://0.0.0.0:4242")
        robot_server_left.run()

        robot_server_right = zerorpc.Server(XArmRobot(ip=""))
        robot_server_right.bind("tcp:// : ")
        robot_server_right.run()

        c = zerorpc.Client()
        c.connect("tcp://127.0.0.1:4242")
        print(c.hello("RPC"))
        # Initialize cameras

        # Action space clipping
        pass

    def step(self, action):
        pass

    def reset(self):
        pass

    def close(self):
        pass

