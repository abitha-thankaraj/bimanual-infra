from torch.utils.data import Dataset
import os

class Policy():
    def __init__(self, env):
        self.env = env
        self.action_space = env.action_space
        self.observation_space = env.observation_space

    def get_action(self, obs):
        raise NotImplementedError

# Pytorch dataloader for robot data
class RobotDataSet(Dataset):
    def __init__(self) -> None:
        pass

    def __getitem__(self, index):
        pass


def load_data(fname):
    pass

def load_trajectory(trajectory_dir):
    traj_files = os.lostdir(trajectory_dir)
    # load obs, act, obs_prime
    pass