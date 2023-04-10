import os
from multiprocessing import Process
import torch, torchvision
import numpy as np

class ImageRecorder:
    def __init__(self, cam= None, save_dir = None):
        super().__init__()
        self.cam = cam
        self.save_dir = save_dir
        self.exit = False
        self.imgs = []

    def run(self):
        while not self.exit:
            bgr, dep = self.cam.get_frame()
            self.imgs.append(torch.Tensor(np.array(bgr)))

    def stop(self):
        self.exit = True
        self.join()
    
    def save_video(self):
        torchvision.io.write_video(str(os.path.join(self.save_dir, 'video.mp4')), torch.stack(self.imgs), 30)

    def clear(self):
        del self.imgs
        self.imgs = []