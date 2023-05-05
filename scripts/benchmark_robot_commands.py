import pandas as pd
from bimanual.hardware.robot import Robot
from bimanual.utils.debug_utils import DebugTimer, benchmark_cmd

def concat_states(robot: Robot, n_states=1000):
    s = robot.get_current_state_action_tuple().to_df()
    for _ in range(n_states):
        s = pd.concat([s, robot.get_current_state_action_tuple().to_df()])


if __name__ == "__main__":
    robot = Robot()
    robot.clear()
    robot.reset()

    n_trials = 10

    benchmark_cmd(n_trials, robot.get_current_state_action_tuple,
                  name="Robot get s,a tuples")
    benchmark_cmd(n_trials, robot.get_servo_angle, name="Robot get js")
    benchmark_cmd(n_trials, robot.get_position_aa, name="Robot get pose")
    benchmark_cmd(n_trials, robot.get_gripper_position,
                  name="Robot get gripper")
    # benchmark_cmd(n_trials, concat_states, kwargs={
    #               "robot": robot}, name="conct states")

    df_0 = robot.get_current_state_action_tuple().to_df()
    df = robot.get_current_state_action_tuple().to_df()
    for _ in range(100):
        df = pd.concat([df, df_0])

    with DebugTimer("df to dict") as dt:
        for _ in range(1):
            df = pd.concat([df, df_0])
    print(dt.interval)
    from IPython import embed
    embed()
