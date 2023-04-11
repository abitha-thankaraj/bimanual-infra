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
        self.speed = speed
        self.acceleration = acceleration

class JointMoveMessage(MoveMessage):
    def __init__(self, target, speed=100, acceleration=100):
        super(JointMoveMessage, self).__init__(target)
        self.speed = speed
        self.acceleration = acceleration


class Robot(XArmAPI):
    def __init__(self, ip, do_not_open=False, 
                 robot_control_mode=RobotControlMode.CARTESIAN_POSITION, 
                 is_radian=True):
        super(Robot, self).__init__(port=ip,
                         is_radian=is_radian,
                         do_not_open=do_not_open)
        
        self._control_mode = robot_control_mode
        
        self._message_queue = mp.Queue()
        self._start_moving_thread = mp.Process(target=self._start_moving)
        self._control_frequency = None # TODO; measure control frequency for each box
        self._control_timeperiod = 1./ self._control_frequency
        
        # Tracking message queue

        # Last message received by the robot
        self._last_received_msg = None # TODO : Implement this as mp object; use BaseManager;
        # Time at which last message received by the robot
        self._last_sent_msg_ts = mp.Value("d", time.time(), lock=False) # Single process access; no need for lock; Makes process faster.
        # Last message sent to the message queue by the calling process
        self._last_queued_msg = None # TODO : Implement this as mp object; use BaseManager;
        

    def reset(self, home = False):
        # TODO : Clean this
        self.clean_error()
        self.clean_warn()
        self.motion_enable(enable=True)
        
        self.set_mode(self._control_mode.value)
        self.set_state(state=0)
    
        if home:
            self.home_robot()
    
    def connect(self):
        self._start_moving_thread.start()
        self.reset(home=True)
    

    def _set_move_command(self):
        if self._control_mode == RobotControlMode.CARTESIAN_POSITION:
            self.move_cmd = self.set_position
        elif self._control_mode == RobotControlMode.JOINT_POSITION:
            self.move_cmd = self.set_servo_angle_j
        else:
            raise NotImplementedError("Not implemented for control mode: {}".format(self._control_mode))

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # Print error message if exception is raised
        if exc_type is not None:
            print(f"Exception raised: {exc_type}, {exc_value}, {traceback}")
        
        if self._start_moving_thread.is_alive():
            self._start_moving_thread.terminate()
            self._start_moving_thread.join()
        
        # TODO: Close context manager

        self.disconnect()

    @property
    def last_sent_target(self):
        return self._last_queued_msg.target
    
    @property
    def last_received_target(self):
        return self._last_received_msg.target
    
    def home_robot(self):
        pass

    def move(self, move_msg):
        # Add target to message queue. 
        # TODO: Should this be an atomic operation (combined update of put and)?
        self._message_queue.put(move_msg)
        self._last_received_msg = move_msg

    def _start_moving(self): #TODO: Add args for _last_sent_msg_ts; 
        while True:
            # Send message to robot at control frequency
            if time.time() - self._last_sent_msg_ts > self._control_timeperiod:
                if not self._message_queue.empty():
                    move_msg = self._message_queue.get()
                    if move_msg.is_terminal():
                        break
                    #TODO : Move to target check for tolerance in step_size. - No; move this to the gym environment.
                    self.move_cmd(**move_msg.target,speed = move_msg.speed, mvacc=move_msg.acceleration,
                                   wait=False, relative = False) #TODO: Add relative flag
                    self._last_sent_msg_ts = time.time()
                    self._last_queued_msg = move_msg
            else:
                time.sleep(self._control_timeperiod - 0.0001)
    
if __name__ == "__main__":
    from IPython import embed; embed()