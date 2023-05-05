from typing import Callable
from bimanual.hardware.robot import Robot
from bimanual.utils.debug_utils import DebugTimer


def benchmark_cmd(n_trials: int, cmd: Callable[[], None], name: str) -> None:
    intervals = []
    for _ in range(n_trials):
        with DebugTimer(name) as ds:
            cmd()
        intervals.append(ds.interval)
    print("Avg time taken for {} : {}".format(
        name, sum(intervals)/len(intervals)))


if __name__ == "__main__":
    robot = Robot()
    robot.clear()
    robot.reset()

    n_trials = 10

    benchmark_cmd(n_trials, robot.get_current_state_action_tuple,
                  "Robot get s,a tuples")
    benchmark_cmd(n_trials, robot.get_servo_angle, "Robot get js")
    benchmark_cmd(n_trials, robot.get_position_aa, "Robot get pose")
    benchmark_cmd(n_trials, robot.get_gripper_position, "Robot get gripper")

    # for _ in range(n_trials):
    #     with DebugTimer("Robot get s,a tuples") as ds:
    #         robot.get_current_state_action_tuple()
    #     print(ds.interval)

    # for _ in range(n_trials):
    #     with DebugTimer("Robot get js"):
    #         robot.get_servo_angle()[1]

    # for _ in range(n_trials):
    #     with DebugTimer("Robot get pose"):
    #         robot.get_position_aa()[1]

    # for _ in range(n_trials):
    #     with DebugTimer("Robot get gripper"):
    #         robot.get_gripper_position()[1]

    from IPython import embed
    embed()
