import time
from typing import Callable


class DebugTimer():
    def __init__(self, name="snippet", return_time_flag=False) -> None:
        self.name = name
        self.return_time_flag = return_time_flag

    def __enter__(self):
        self.start = time.time()
        return self

    def __exit__(self, *args):
        self.end = time.time()
        self.interval = self.end - self.start
        # if self.return_time_flag:
        #     return self.interval
        # print(f"Time elapsed for {self.name}: {self.interval:.6f}s")


def benchmark_cmd(n_trials: int, cmd: Callable[[], None], kwargs: dict = dict(), name: str = None) -> None:
    intervals = []
    for _ in range(n_trials):
        with DebugTimer(name) as ds:
            cmd(**kwargs)
        intervals.append(ds.interval)
    print("Avg time taken for {} : {}".format(
        name, sum(intervals)/len(intervals)))
