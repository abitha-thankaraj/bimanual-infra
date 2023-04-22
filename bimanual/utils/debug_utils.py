import time

class DebugTimer():
    def __init__(self, name ="snippet", return_time_flag =False) -> None:
        self.name = name
        self.return_time_flag = return_time_flag 

    def __enter__(self):
        self.start = time.time()
        return self
    
    def __exit__(self, *args):
        self.end = time.time()
        self.interval = self.end - self.start
        print(f"Time elapsed for {self.name}: {self.interval:.6f}s")
        if self.return_time_flag:
            return self.interval