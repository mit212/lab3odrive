#!/usr/bin/python

# 2.12 Lab 3 trajectory planning
# Jerry Ng Feb 2020

import rospy
import planner
import std_msgs.msg, sensor_msgs.msg
import numpy as np
from OdriveClass import Odrive

USE_REAL_ARM = False
odrive_id = [''] #Odrive SN
full_rev = 400000 #Number of counts in a full revolution
CURR_THRESHOLD = 2
class OdriveController:
	def __init__(self, odrive_ids):
		#Assumes ODrives are used in order of first link then second link
		self.full_revolution = 400000 #counts per revolution
		self.odrive_ctrl = Odrive(odrive_id)
		self.axes = self.odrive_ctrl.axis

		rospy.init_node("odrives")
		self.odrive_ctrl.startup_init()

		#After everything is connected, zero encoder values
		self.ctrl_limit = limit_finder(self.odrive_ctrl,self.axes[0], 0)
		#self.ctrl_limit = limit_finder(self.odrive_ctrl,self.axes[1], 1)


		# rospy.Subscriber('/joint1_controller/position', std_msgs.msg.Float64, axis1_callback)
		# rospy.Subscriber('/joint2_controller/position', std_msgs.msg.Float64, axis2_callback)

		# #Publishers for ready signal
		# self.pos1_pub = rospy.Publisher('/joint1_controller/position', std_msgs.msg.Float64, queue_size=1)
		# self.vel1_pub = rospy.Publisher('/joint1_controller/velocity', std_msgs.msg.Float64, queue_size=1)
		# self.curr1_pub = rospy.Publisher('/joint1_controller/current', std_msgs.msg.Float64, queue_size=1)
		# self.err1_pub = rospy.Publisher('/joint1_controller/error', std_msgs.msg.Boolean, queue_size=1)
		
		# self.pos2_pub = rospy.Publisher('/joint2_controller/position', std_msgs.msg.Float64, queue_size=1)
		# self.vel2_pub = rospy.Publisher('/joint2_controller/velocity', std_msgs.msg.Float64, queue_size=1)
		# self.curr2_pub = rospy.Publisher('/joint2_controller/current', std_msgs.msg.Float64, queue_size=1)
		# self.err2_pub = rospy.Publisher('/joint2_controller/error', std_msgs.msg.Boolean, queue_size=1)

		# self.pos1 = std_msgs.msg.Float64()
		# self.vel1 = std_msgs.msg.Float64()
		# self.curr1 = std_msgs.msg.Float64()
		# self.err1 = std_msgs.msg.Boolean()

		# self.pos2 = std_msgs.msg.Float64()
		# self.vel2 = std_msgs.msg.Float64()
		# self.curr2 = std_msgs.msg.Float64()
		# self.err2 = std_msgs.msg.Boolean()

		# #No Error at start
		# self.err1.data = False
		# self.err2.data = False
		while not rospy.is_shutdown():
			# #Publish current, velocity and etc for both axes
			# self.check_val(axis = 0)
			# self.check_val(axis = 1)
			# self.pub1()
			# self.pub2()
			rospy.sleep(0.01)

	def limit_finder(self, ctrl):
		#Run encoder calibration by moving in the negative direction
		
		for i in range(0,2):
			ctrl.VelMove(vel_setpt = -8000, num = i)
			home = False
			while not home:
				if(abs(self.axes[i].motor.current_control.Iq_measured) > CURR_THRESHOLD):
					self.axes[i].controller.vel_setpoint = 0
					home = True
				rospy.sleep(0.01)

		return [self.axes[0].encoder.pos_estimate, self.axes[1].encoder.pos_estimate]

	def check_limits(self,motor_location, set_pos):
		#Return True if feasible
		#Feasible if greater than the zeroing position and less than 1/3 of a revolution away from zero
		if (set_pos > self.ctrl_limit[motor_location-1]) and (set_pos < self.ctrl_limit[motor_location-1] + 3*full_rev/4):
			return True
		else:
			return False

	def check_val(self,axis):
		if axis == 1:
			self.pos1.data = self.axes[axis].encoder.pos_estimate
			self.vel1.data = self.axes[axis].encoder.vel_estimate
			self.curr1.data = self.axes[axis].motor.current_control.Iq_measured
			if self.axes[axis].error != 0 or self.axes[axis].motor.error != 0 or self.axes[axis].encoder.error != 0:
				self.err1.data = True
		elif axis == 2:
			self.pos2.data = self.axes[axis].encoder.pos_estimate
			self.vel2.data = self.axes[axis].encoder.vel_estimate
			self.curr2.data = self.axes[axis].motor.current_control.Iq_measured
			if self.axes[axis].error != 0 or self.axes[axis].motor.error != 0 or self.axes[axis].encoder.error != 0:
				self.err2.data = True

	def axis1_callback(self, data):
		#Run motion for axis 1
		set_pos_global = RAD2CNT(data.data)
		feasible = check_limits(1,set_pos)
		if(feasible):
			set_pos = set_pos_global - self.ctrl_limit[0]
			self.odrive_ctrl.trajMoveCnt(0, posDesired = set_pos)
		else:
			#Do nothing if its not feasible
			print('Unfeasible location sent to motor 1')

	def axis2_callback(self, data):
		#Run motion for axis 2
		set_pos = RAD2CNT(data.data)
		feasible = check_limits(2,set_pos)
		if(feasible):
			set_pos = set_pos_global - self.ctrl_limit[1]
			self.odrive_ctrl.trajMoveCnt(1, posDesired = set_pos)
		else:
			#Do nothing if its not feasible
			print('Unfeasible location sent to motor 2')

	def RAD2CNT(self, angle):
		return np.rint(angle%(2*np.pi) * self.full_rev)

if __name__ == '__main__':

	if(USE_REAL_ARM):
		controller = OdriveController(odrive_ids, odrive_axes)
		rospy.spin()

	else:
		exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)


