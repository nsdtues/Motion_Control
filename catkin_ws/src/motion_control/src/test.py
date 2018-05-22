#!/usr/bin/env python

import rospy
import os
# from motion_control.msg import sys_cmd_msg_to_motor
from motion_control.msg import motion_module_defualt_para
from config import Parameter as para
import datetime
import zmq


class GET_PARA(object):
    def __init__(self):
		rospy.init_node('get_para_node')
		# self.pub_cmd = rospy.Publisher('sys_cmd_msg_to_motor',sys_cmd_msg_to_motor,queue_size=1)
		self.pub_para = rospy.Publisher('defualt_para',motion_module_defualt_para,queue_size=1)
		
		# configFileName = "config.py"
		
		# try:
			# if os.path.exists(configFileName):
				# rospy.loginfo("can open config file\n")
				# from configFileName import Parameter as para
		# except ImportError:
				# rospy.loginfo("can open config file\n")
		
		para_msg = motion_module_defualt_para()
		para_msg.foot = para["foot"]
		para_msg.forceaid = para["forceaid"]
		para_msg.max_force = para["max_force"]
		para_msg.max_position = para["max_position"]
		para_msg.zero_position = para["zero_position"]
		para_msg.preload_position = para["preload_position"]
		para_msg.max_velocity = para["max_velocity"]
		para_msg.nset_acc = para["nset_acc"]
		para_msg.max_pot = para["max_pot"]
		para_msg.pid_kp = para["pid_kp"]
		para_msg.pid_ki = para["pid_ki"]
		para_msg.pid_umax = para["pid_umax"]
		para_msg.pid_umin = para["pid_umin"]	
		rospy.loginfo("config para {}".format(para_msg))
		self.pub_para.publish(para_msg)

		# bind to gait stream
		self.fixed_gait_stream = "tcp://*:8011"
		self.fixed_gait_context = zmq.Context()
		self.fixed_gait_socket = self.fixed_gait_context.socket(zmq.PUB)
		self.fixed_gait_socket.bind(self.fixed_gait_stream)
		
		rate = rospy.Rate(100) # 100Hz
		while not rospy.is_shutdown():
			# cmd_msg = sys_cmd_msg_to_motor()
			# cmd_msg.syscmd = 'cmdmotorinitial'
			# self.pub_cmd.publish(cmd_msg)
			self.gait = "GaitL:A, GaitR:B, Gait:GaitWalking"
			self.fixed_gait_socket.send_string(self.gait)		
			now = datetime.datetime.now()
			print("{} ".format(now) + self.gait)				
			# now = datetime.datetime.now()
			# print(now.microsecond)
			rate.sleep()

			
if __name__ == '__main__':
    GET_PARA()

