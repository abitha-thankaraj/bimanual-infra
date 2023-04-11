from enum import Enum
from xarm.wrapper import XArmAPI
import multiprocessing as mp
import time

class RobotControlMode(Enum):
    CARTESIAN_POSITION = 0
    JOINT_POSITION = 1

class MoveMessage():
    def __init__(self, target):
        self.target = target
        self.created_timestamp = time.time()

    def is_terminal(self): # Sentinal message
        return self.target is None

class CartesianMoveMessage(MoveMessage):
    def __init__(self, target, speed=100, acceleration=100):
        super(CartesianMoveMessage, self).__init__(target)

class JointMoveMessage(MoveMessage):
    def __init__(self, target, speed=100, acceleration=100):
        super(JointMoveMessage, self).__init__(target)


class Robot(XArmAPI):
    def __init__(self, ip, port=0, do_not_open=False, 
                 robot_control_mode=RobotControlMode.CARTESIAN_POSITION, is_radian=False):
        super(Robot, self).__init__(ip, port, do_not_open, is_radian)
        self._control_mode = robot_control_mode
        
        self._message_queue = mp.Queue()
        self._start_moving_thread = mp.Process(target=self._start_moving)
        self._control_frequency = None # TODO; measure control frequency for each box
        self._control_timeperiod = 1./ self._control_frequency
        
        # Tracking message queue
        self._last_received_msg = None # TODO : Implement this as mp object?
        self._last_sent_msg = None    
        self._last_sent_message_ts = mp.Value("d", time.time(), lock=False) # Single process access; no need for lock; Makes process faster.

    def reset(self, home = False):
        #TODO : Clean this
        self.clean_error()
        self.clean_warn()
        self.motion_enable(enable=True)
        
        self.set_mode(self._control_mode.value)
        self.set_state(state=0)
    
        if home:
            self.home_robot()
    

    def __exit__(self, exc_type, exc_value, traceback):
        # Print error message if exception is raised
        if exc_type is not None:
            print(f"Exception raised: {exc_type}, {exc_value}, {traceback}")
        
        if self._start_moving_thread.is_alive():
            self._start_moving_thread.terminate()
            self._start_moving_thread.join()        
        self.disconnect()


    def home_robot(self):
        pass

    def move(self, move_msg):
        # Add target to message queue
        self._message_queue.put(move_msg)

    def _start_moving(self):
        while True:
            # Send message to robot at control frequency
            if time.time() - self._last_sent_message_ts > self._control_timeperiod:
                if not self._message_queue.empty():
                    move_msg = self._message_queue.get()
                    if move_msg.is_terminal():
                        break
                    #TODO :Move to target check for tolernace in step_size. 
                    self._move_to_target(move_msg)
                    self._last_sent_message_ts = time.time()
                    self._last_sent_msg = move_msg
            else:
                time.sleep(self._control_timeperiod - 0.0001)
    
    def _move_to_target(self, target):
        pass

if __name__ == "__main__":
    from IPython import embed; embed()