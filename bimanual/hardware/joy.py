import time
import pygame
import argparse

# from bimanual.hardware.robot import XArm




import sys
import time
import numpy as np
import random
from configparser import ConfigParser
from xarm.wrapper import XArmAPI

class XArm:

	def __init__(self,ip="192.168.86.230", home_displacement = (0,0,0), low_range=(1,1,0.2) , high_range=(2,2,1),
				 keep_gripper_closed=False, highest_start=False, x_limit=None, y_limit=None, z_limit=None, pitch = 0, roll=180, gripper_action_scale=200):
		self.arm = XArmAPI(ip)
		self.gripper_max_open = 600
		self.gripper_min_open = 0 #348
		self.zero = (206/100,0/100,120.5/100)	# Units: .1 meters 
		self.home = home_displacement
		self.keep_gripper_closed = keep_gripper_closed
		self.highest_start = highest_start
		self.low_range = low_range
		self.high_range = high_range
		self.joint_limits = None
		self.ip = ip
		self.gripper_action_scale = gripper_action_scale

		# Limits
		self.x_limit = [0.5, 3.5] if x_limit is None else x_limit
		self.y_limit = [-1.7, 1.3] if y_limit is None else y_limit
		self.z_limit = [1.4, 3.4] if z_limit is None else z_limit # PressBlock

		# Pitch value - Horizontal or vertical orientation
		self.pitch = pitch
		self.roll = roll

	def start_robot(self):
		if self.ip is None:
			raise Exception('IP not provided.')
		self.arm = XArmAPI(self.ip, is_radian=False)
		self.arm.motion_enable(enable=False)
		self.arm.motion_enable(enable=True)
		if self.arm.error_code != 0:
			self.arm.clean_error()
		self.set_mode_and_state()

	def set_mode_and_state(self, mode=0, state=0):
		self.arm.set_mode(mode)
		self.arm.set_state(state=state)

	def clear_errors(self):
		self.arm.clean_warn()
		self.arm.clean_error()

	def has_error(self):
		return self.arm.has_err_warn

	def reset(self, home = False, reset_at_home=True):
		if self.arm.has_err_warn:
			self.clear_errors()
		if home:
			if reset_at_home:
				self.move_to_home()
			else:
				self.move_to_zero()
			if self.keep_gripper_closed:
				self.close_gripper_fully()
			else:
				self.open_gripper_fully()

	def move_to_home(self, open_gripper=False):
		pos = self.get_position()
		pos[0] = self.home[0]
		pos[1] = self.home[1]
		pos[2] = self.home[2]
		self.set_position(pos)
		if open_gripper and not self.keep_gripper_closed:
			self.open_gripper_fully()

	def move_to_zero(self):
		pos = self.get_position()
		pos[0] = min(max(self.x_limit[0],0), self.x_limit[1])# 0
		pos[1] = min(max(self.y_limit[0],0), self.y_limit[1])# 0
		pos[2] = min(max(self.z_limit[0],0), self.z_limit[1]) if not self.highest_start else self.z_limit[1] # 0
		self.set_position(pos)

	def set_position(self, pos, wait=False):
		pos = self.limit_pos(pos)
		x = (pos[0] + self.zero[0])*100
		y = (pos[1] + self.zero[1])*100
		z = (pos[2] + self.zero[2])*100
		self.arm.set_position(x=x, y=y, z=z, roll=self.roll, pitch=self.pitch, yaw=0, wait=wait)

	def get_position(self):
		pos = self.arm.get_position()[1]
		x = (pos[0]/100.0 - self.zero[0])
		y = (pos[1]/100.0 - self.zero[1])
		z = (pos[2]/100.0 - self.zero[2])
		return np.array([x,y,z, pos[3], pos[4], pos[5]]).astype(np.float32)

	def get_gripper_position(self):
		code, pos = self.arm.get_gripper_position()
		if code!=0:
			raise Exception('Correct gripper angle cannot be obtained.')
		return pos

	def open_gripper_fully(self):
		self.set_gripper_position(self.gripper_max_open)

	def close_gripper_fully(self):
		self.set_gripper_position(self.gripper_min_open)

	def open_gripper(self):
		self.set_gripper_position(self.get_gripper_position() + self.gripper_action_scale)

	def close_gripper(self):
		self.set_gripper_position(self.get_gripper_position() - self.gripper_action_scale)

	def set_gripper_position(self, pos, wait=False):
		'''
		wait: To wait till completion of action or not
		'''
		if pos<self.gripper_min_open:
			pos = self.gripper_min_open
		if pos>self.gripper_max_open:
			pos = self.gripper_max_open
		self.arm.set_gripper_position(pos, wait=wait, auto_enable=True)

	def get_servo_angle(self):
		code, angles = self.arm.get_servo_angle()
		if code!=0:
			raise Exception('Correct servo angles cannot be obtained.')
		return angles

	def set_servo_angle(self, angles, is_radian=None):
		'''
		angles: List of length 8
		'''
		self.arm.set_servo_angle(angle=angles, is_radian=is_radian)
	
	def limit_pos(self, pos):
		pos[0] = max(self.x_limit[0], pos[0])
		pos[0] = min(self.x_limit[1], pos[0])
		pos[1] = max(self.y_limit[0], pos[1])
		pos[1] = min(self.y_limit[1], pos[1])
		pos[2] = max(self.z_limit[0], pos[2])
		pos[2] = min(self.z_limit[1], pos[2])
		return pos


class Joy:

	def __init__(self,
				 arm,
				 joy_id=0,
				 scale_factor_pos=0.3, 
				 scale_factor_gripper=300, 
				 scale_factor_rotation=120, #0.1,
				 motion_scale_change=0.01,
				 random_start=False,
				 sleep=0.6,
				 is_radian=False):

		pygame.init()
		self.joy = pygame.joystick.Joystick(joy_id)
		self.joy.init()
		self.goal_pos = [1.73,0.09,0.2]
		self.global_mov = [-1,5,0]
		self.random_start = random_start
		
		self.arm = arm
		self.scale_factor_pos = scale_factor_pos
		self.scale_factor_gripper = scale_factor_gripper
		self.scale_factor_rotation = scale_factor_rotation
		self.motion_scale_change = motion_scale_change
		self.sleep = sleep
		self.is_radian = is_radian

		self.forward = False
		self.backward = False
		self.left = False
		self.right = False
		self.up = False
		self.down = False
		self.close_gripper = False
		self.open_gripper = False
		self.start = False
		self.stop = False
		self.cancel = False
		self.go_home = False
		self.bring_arm_up = False
		self.bring_arm_down = False
		self.rotate_arm_cw = False
		self.rotate_arm_ccw = False
		self.motion_scale = 0

	def init_arm(self, reset=True):
		self.arm = XArm()
		self.arm.start_robot()

		if reset:
			self.arm.reset(home=True)
			time.sleep(2)

	def detect_event(self):
		events = pygame.event.get()
		for event in events:
			if event.type == pygame.JOYAXISMOTION:
				if event.axis==1 and event.value>0.5:
					self.forward = True
				elif event.axis==1 and event.value<-0.5:
					self.backward = True
				elif event.axis==1:
					self.forward = False
					self.backward = False
				elif event.axis==0 and event.value>0.5:
					self.left = True
				elif event.axis==0 and event.value<-0.5:
					self.right = True
				elif event.axis==0:
					self.left = False
					self.right = False
				elif event.axis==4 and event.value<-0.5:
					self.up = True
				elif event.axis==4 and event.value>0.5:
					self.down = True
				elif event.axis==4:
					self.up = False
					self.down = False
				elif event.axis==3 and event.value<-0.5:
					self.rotate_arm_cw = True
				elif event.axis==3 and event.value>0.5:
					self.rotate_arm_ccw = True
				elif event.axis==3:
					self.rotate_arm_cw = False
					self.rotate_arm_ccw = False
				

			if event.type == pygame.JOYBUTTONDOWN:
				if event.button == 0:			# A
					print("Close gripper")
					self.close_gripper = True
				elif event.button == 1:			# B
					print("Open gripper")
					self.open_gripper = True
				elif event.button == 7:			# start
					print("Start")
					self.start = True
				elif event.button == 6:			# back
					print("Stop")
					self.stop = True
				elif event.button == 2:			# X
					print("Go Home")
					self.go_home = True
				elif event.button == 3:			# Y
					print("Cancel demo")
					self.cancel = True
				elif event.button == 4:			# LB
					print("Decrease motion scale")
					self.scale_factor_pos = max(0.01, self.scale_factor_pos-self.motion_scale_change)
				elif event.button == 5:			# RB
					print("Increase motion scale")
					self.scale_factor_pos += self.motion_scale_change
			
			elif event.type == pygame.JOYHATMOTION:
				if event.hat == 0 and event.value == (0,1):
					print("Reset arm")
					self.arm.reset(home=True)

	def move(self):
		if self.arm.has_error():
			self.arm.clear_errors()
			self.arm.set_mode_and_state()
			self._move_up()
		pos, action = None, None
		if self.forward:
			pos = self._move_forward()
			action = 'move_forward'
			self.global_mov[0] += 1
			print(f"global_mov:{self.global_mov}")
		elif self.backward:
			pos = self._move_backward()
			action = 'move_backward'
			self.global_mov[0] -= 1
			print(f"global_mov:{self.global_mov}")
		elif self.left:
			pos = self._move_left()
			action = 'move_left'
			self.global_mov[1] += 1
			print(f"global_mov:{self.global_mov}")
		elif self.right:
			pos = self._move_right()
			action = 'move_right'
			self.global_mov[1] -= 1
			print(f"global_mov:{self.global_mov}")
		elif self.up:
			pos = self._move_up()
			action = 'move_up'
			self.global_mov[2] += 1
			print(f"global_mov:{self.global_mov}")
		elif self.down:
			pos = self._move_down()
			action = 'move_down'
			self.global_mov[2] -= 1
			print(f"global_mov:{self.global_mov}")
		elif self.close_gripper:
			pos = self._close_gripper()
			self.close_gripper = False
			action = 'close_gripper'
			print(f"global_mov:{self.global_mov}")
		elif self.open_gripper:
			pos = self._open_gripper()
			self.open_gripper = False
			action = 'open_gripper'
			print(f"global_mov:{self.global_mov}")
		elif self.go_home:
			pos = self.arm.move_to_home(open_gripper=False)
			self.go_home = False
			action = 'go_home'
			self.global_mov = [-1,5,0]
			print(f"global_mov:{self.global_mov}")
		elif self.start:
			pos = None
			action = 'start'
			self.start = False
		elif self.stop:
			pos = None
			action = 'stop'
			self.stop = False
		elif self.cancel:
			pos = None
			action = 'cancel'
			self.cancel = False
		elif self.rotate_arm_cw:
			pos = self._rotate_arm_cw()
			action = 'rotate_arm_cw'
		elif self.rotate_arm_ccw:
			pos = self._rotate_arm_ccw()
			action = 'rotate_arm_ccw'


		return pos, action		
	
	def _move_forward(self):
		pos = self.arm.get_position()
		pos[0] += self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos
		
	def _move_backward(self):
		pos = self.arm.get_position()
		pos[0] -= self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos

	def _move_left(self):
		pos = self.arm.get_position()
		pos[1] += self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos	

	def _move_right(self):
		pos = self.arm.get_position()
		pos[1] -= self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)	
		return pos	

	def _move_up(self):
		pos = self.arm.get_position()
		pos[2] += self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos

	def _move_down(self):
		pos = self.arm.get_position()
		pos[2] -= self.scale_factor_pos
		self.arm.set_position(pos)
		time.sleep(self.sleep)
		return pos

	def _open_gripper(self):
		self.arm.open_gripper_fully()
		pos = 1
		time.sleep(self.sleep/2)
		return pos	

	def _close_gripper(self):
		self.arm.close_gripper_fully()
		pos = -1
		time.sleep(self.sleep/2)
		return pos

	def _bring_arm_up(self):
		angles = self.arm.get_servo_angle()
		angles[5] -= self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[5]

	def _bring_arm_down(self):
		angles = self.arm.get_servo_angle()
		angles[5] += self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[5]

	def _rotate_arm_cw(self):
		angles = self.arm.get_servo_angle()
		angles[6] -= self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[6]

	def _rotate_arm_ccw(self):
		angles = self.arm.get_servo_angle()
		angles[6] += self.scale_factor_rotation
		self.arm.set_servo_angle(angles, is_radian=self.is_radian)
		time.sleep(self.sleep)
		return angles[6]

if __name__ == "__main__":
		import argparse
		parser = argparse.ArgumentParser()
		parser.add_argument("--ip", type=str, default="192.168.86.230")
		parser.add_argument("--joyid", type=int, default=0)
		args = parser.parse_args()
		

		pygame.init()
		
		joy = Joy(arm=XArm(ip=args.ip),
	    		joy_id=args.joyid,
	    		scale_factor_pos=0.15,
				scale_factor_gripper=50, 
				scale_factor_rotation=120,
				motion_scale_change=0.03,
				sleep=0.4)
		while(True):
			joy.detect_event()
			pos, action = joy.move()


# python joy.py --ip=192.168.86.216
# python joy.py --joyid=1