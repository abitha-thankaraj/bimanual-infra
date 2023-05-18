import pandas as pd
import numpy as np
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
class RobotTrajectoryDataSet(Dataset):
    def __init__(self, data_dir) -> None:
        self.data_dir = data_dir
        
        # Get all csv files in data_dir
        fnames = os.listdir(data_dir)
        fnames = [os.path.join(data_dir, fname) for fname in fnames if fname.endswith(".csv")]
        
        self.fnames = fnames
        
    def __getitem__(self, index):
        # Load individual trajectory from csv file
        obs, acts = load_data(self.fnames[index])

        pass


def load_data(fname):
    df = pd.read_csv(fname)

    obs = np.array([k.strip("[").strip("]").split() for k in df['pose_aa'].to_numpy()], dtype=float)
    acts = obs[1:] 
    acts.append(np.zeros(6))

    return (obs, acts)


def load_trajectory(trajectory_dir):
    traj_files = os.listdir(trajectory_dir)

    # load obs, act, obs_prime
    pass