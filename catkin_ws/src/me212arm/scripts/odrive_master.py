#!/usr/bin/python

# 2.12 Lab 3 trajectory planning
# Jerry Ng Feb 2020

import rospy
import planner
import std_msgs.msg, sensor_msgs.msg
import numpy as np
from OdriveClass import Odrive

USE_REAL_ARM = False
odrive_ids = [''] #base odrive, then secondary odrive
odrive_axes = [] #base axis number, then second axis number
full_rev = 400000 #Number of counts in a full revolution
CURR_THRESHOLD = 2
class OdriveController:
	def __init__(self, odrive_ids, odrive_axes):
		#Assumes ODrives are used in order of first link then second link
		self.full_revolution = 400000 #counts per revolution
		for i in range(0,len(odrive_ids)):
			self.odrive_ctrl[i] = Odrive(odrive_ids(str(i)),odrive_axes(i))
			self.axes[i] = self.odrive_ctrl[i].axes[odrive_axes[i]]

		self.axes_num = odrive_axes

		#After everything is connected, zero encoder values
		for i in range(0,len(self.odrive_ctrl)):
			self.ctrl_limit[i] = limit_finder(self.odrive_ctrl[i],self.axes[i], self.axes_num[i])

		rospy.Subscriber('/joint1_controller/position', std_msgs.msg.Float64, axis1_callback)
		rospy.Subscriber('/joint2_controller/position', std_msgs.msg.Float64, axis2_callback)

		# #Publishers for ready signal
		# self.joint1_pub = rospy.Publisher('/joint1_controller/value', std_msgs.msg.Float64, queue_size=1)
		# self.joint2_pub = rospy.Publisher('/joint2_controller/value', std_msgs.msg.Float64, queue_size=1)

	def limit_finder(self, ctrl, axis, num):
		#Run encoder calibration by moving in the negative direction
		ctrl.VelMove(vel_setpt = -8000, num = num)
		home = False
		while not home:
			if(abs(axis.motor.current_control.Iq_measured) > CURR_THRESHOLD):
				axis.controller.vel_setpoint = 0
				home = True
			rospy.sleep(0.01)

		return axis.encoder.pos_estimate

	def check_limits(self,motor_location, set_pos):
		#Return True if feasible
		#Feasible if greater than the zeroing position and less than 1/3 of a revolution away from zero
		if (set_pos > self.ctrl_limit[motor_location-1]) and (set_pos < self.ctrl_limit[motor_location-1] + 3*full_rev/4):
			return True
		else:
			return False

	def axis1_callback(self, data):
		#Run motion for axis 1
		set_pos = RAD2CNT(data.data)
		feasible = check_limits(1,set_pos)
		if(feasible):
			self.odrive_ctrl[0].trajMoveCnt(self.axes_num[0], posDesired = set_pos)
		else:
			#Do nothing if its not feasible
			print('Unfeasible location sent to motor 1')

	def axis2_callback(self, data):
		#Run motion for axis 2
		set_pos = RAD2CNT(data.data)
		feasible = check_limits(2,set_pos)
		if(feasible):
			self.odrive_ctrl[1].trajMoveCnt(self.axes_num[1], posDesired = set_pos)
		else:
			#Do nothing if its not feasible
			print('Unfeasible location sent to motor 2')

	def RAD2CNT(self, angle):
		return np.rint(angle%(2*np.pi) * self.full_rev)

if __name__ == '__main__':

	if(USE_REAL_ARM):
		controller = OdriveController(odrive_ids, odrive_axes)

	else:
		exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)


