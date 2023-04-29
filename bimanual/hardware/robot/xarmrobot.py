from enum import Enum
from xarm.wrapper import XArmAPI
import multiprocessing as mp
import time
from multiprocessing.managers import BaseManager
from ctypes import c_double


class RobotControlMode(Enum):
    CARTESIAN_CONTROL = 0
    SERVO_CONTROL = 1

#--------------------------------------------------------------------------------#

class MoveMessage():
    def __init__(self, target):
        self.target = target
        self.created_timestamp = time.time()

    def is_terminal(self): # Sentinal message
        return self.target is None

class CartesianMoveMessage(MoveMessage):
    def __init__(self, target, speed=100, acceleration=100, relative=False, 
                 wait = True, is_radian=True, rotation = None):
        super(CartesianMoveMessage, self).__init__(target)
        self.speed = speed
        self.mvacc = acceleration
        self.relative = relative
        self.wait = wait # Default async calls; do not wait for robot to finish moving.
        self.is_radian = is_radian
        self.rotation = rotation

class GripperMoveMessage(MoveMessage):
    def __init__(self, target, wait = False):
        super(GripperMoveMessage, self).__init__(target)
        self.wait = wait

#--------------------------------------------------------------------------------#


class CustomManager(BaseManager): 
    pass

def register_ctx():
    CustomManager.register('MoveMessage', MoveMessage)
    CustomManager.register('CartesianMoveMessage', CartesianMoveMessage)


#--------------------------------------------------------------------------------#



class Robot(XArmAPI):
    def __init__(self, ip, do_not_open=False, 
                 robot_control_mode=RobotControlMode.CARTESIAN_CONTROL, 
                 is_radian=True, home_target = [206.0, -0.0, 120.5, 3.141593, -0.0, 0.0]):
        super(Robot, self).__init__(port=ip,
                         is_radian=is_radian,
                         do_not_open=do_not_open)
        
        self._control_mode = robot_control_mode

        # register_ctx() # TODO: ensure that register_ctx() is called. Move this outside robot class.
        
        self._ctx_manager = CustomManager(ctx=mp.get_context())
        self._ctx_manager.start()
        # Tracking message queue

        # Last message received by the robot
        self._last_received_msg = self._ctx_manager.CartesianMoveMessage(home_target, relative =False) 
        # TODO : Implement this as mp object; use BaseManager;
        # Time at which last message received by the robot
        self._last_sent_msg_ts = mp.Value("d", time.time(), lock=False) # Single process access; no need for lock; Makes process faster.
        # Last message sent to the message queue by the calling process
        self._last_queued_msg = self._ctx_manager.CartesianMoveMessage(home_target, relative =False) # TODO : Implement this as mp object; use BaseManager;


        self._message_queue = mp.Queue()
        self._start_moving_process = mp.Process(target=self._start_moving, args=(self._message_queue, self._last_sent_msg_ts, self._last_received_msg, self), name = "move_proc")
        self._control_frequency = 1 # TODO; measure control frequency for each box. COmmands should be sent at 30-250. If <30 - choppy; if>250 drop frames. Deatils in manual:
        self._control_timeperiod = 1./self._control_frequency

        self.home_target = home_target
        
        self._set_move_command()

    def reset(self, home = False):
        # TODO : Clean this
        self.clean_error()
        self.clean_warn()
        self.motion_enable(enable=False)
        self.motion_enable(enable=True)
        
        self.set_mode(self._control_mode.value)
        # self.set_mode(1)

        self.set_state(state=0)
    
        if home:
            self.home_robot()
    
    def connect(self):
        #TODO: Start all mp processes and context managed objects here.
        self._start_moving_process.start()
        # self.reset(home=False)
    
    def _set_move_command(self):
        if self._control_mode == RobotControlMode.CARTESIAN_CONTROL:
            self.move_cmd = self.set_position

            # self.move_cmd = self.set_servo_cartesian
        elif self._control_mode == RobotControlMode.SERVO_CONTROL:
            raise NotImplementedError("Not implemented for control mode: {}".format(self._control_mode))
            # self.move_cmd = self.set_servo_angle_j #TODO: Verify this
        else:
            raise NotImplementedError("Not implemented for control mode: {}".format(self._control_mode))

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # Print error message if exception is raised
        if exc_type is not None:
            print(f"Exception raised: {exc_type}, {exc_value}, {traceback}")
        
        if self._start_moving_process.is_alive():
            self._start_moving_process.terminate()
            self._start_moving_process.join()
        
        self._ctx_manager.shutdown()
        self.disconnect()

    @property
    def last_queued_target(self):
        return self._last_queued_msg.target
    
    @property
    def last_received_target(self):
        return self._last_received_msg.target
    
    def home_robot(self):
        self.move(MoveMessage(self.home_target))

    def stop_robot(self):
        self.move(MoveMessage(None))

    def move(self, move_msg):
        # Add target to message queue. 
        # TODO: Should this be an atomic operation (combined update of put and)?
        print("Adding message to queue")
        print(type(move_msg))
        self._message_queue.put(move_msg)
        self._last_queued_msg = move_msg

    def _start_moving(self, queue, last_sent_msg_ts, last_received_msg, bot): #TODO: Add args for _last_sent_msg_ts; 

        bot.reset(home=False)
        
        while True:
            # Send message to robot at control frequency
            if time.time() - last_sent_msg_ts.value > self._control_timeperiod:
               
                if not queue.empty():
                    print("Sending message to robot: {}".format(queue.qsize()))
                    move_msg = queue.get()
                    if move_msg.is_terminal():
                        break
                    #TODO : Move to target check for tolerance in step_size. - No; move this to the gym environment.
                    # print(vars(self))
                    print(move_msg.target)
                    print(vars(move_msg))

                    y = bot.get_position()
                    print("Robot position: {}".format(y))

                    # x= self.set_position(*[216.0, 0.0, 120.5, -3.141592, -0.0, 0.0])
                    # x = self.set_position(*move_msg.target, relative=True, wait=True)

                    # x = self.set_position(*move_msg.target, **vars(move_msg))

                    # x = self.move_cmd(*move_msg.target, **vars(move_msg))
                    # print("Robot command returned: {}".format(x))
                    # self.move_cmd(move_msg.target, **vars(move_msg))
                    

                    last_sent_msg_ts = c_double(time.time())
                    # print(last_received_msg.target)
                    last_received_msg = move_msg

            else:
                time.sleep(0.001)
    
if __name__ == "__main__":
    register_ctx()


    # target_poses = [[206.0, 0.0, 120.5, -3.141592, -0.0, 0.0],
    #                 [216.0, 0.0, 120.5, -3.141592, -0.0, 0.0],
    #                 [226.0, 0.0, 120.5, -3.141592, -0.0, 0.0],
    #                 [236.0, 0.0, 120.5, -3.141592, -0.0, 0.0]]


    # targets = [CartesianMoveMessage(target_pose) for target_pose in target_poses]
    targets = [CartesianMoveMessage([10,0,0,0,0,0], relative=True) for _ in range(4)]
    # targets.extend([CartesianMoveMessage([0,10,0,0,0,0], relative=True) for _ in range(4)])
    # targets.extend([CartesianMoveMessage([-10,0,0,0,0,0], relative=True) for _ in range(4)])
    # targets.extend([CartesianMoveMessage([0,-10,0,0,0,0], relative=True) for _ in range(4)])

    bot = Robot(ip='192.168.86.216')
    bot.connect()
    from IPython import embed; embed()

    start_times = []
    end_times = []
    for target in targets:
        start_times.append(time.time())
        bot.move(target)
        end_times.append(time.time())
    
    from IPython import embed; embed()
