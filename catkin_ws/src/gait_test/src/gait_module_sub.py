#!/usr/bin/env python

import rospy
import os
import datetime
import zmq



class GAIT_MODULE_SUB(object):
    def __init__(self):
		rospy.init_node('fixed_gait_sub')
		
		self.loop()
		
    def loop(self):	
		

		self.gait_module_stream = "tcp://localhost:8011"
		self.gait_module_context = zmq.Context()
		self.gait_module_socket = self.gait_module_context.socket(zmq.SUB)
		self.gait_module_socket.connect(self.gait_module_stream)
		
		self.dataFilter = ''
		if isinstance(self.dataFilter, bytes):
			self.dataFilter = self.dataFilter.decode('ascii')

		self.gait_module_socket.setsockopt_string(zmq.SUBSCRIBE, self.dataFilter)		
		
		while not rospy.is_shutdown():
			self.gaitstr = self.gait_module_socket.recv_string()
			self.now = datetime.datetime.now()
			print("sub {}, ".format(self.now) + self.gaitstr)	


if __name__ == '__main__':
    try:
        GAIT_MODULE_SUB()
    except KeyboardInterrupt:
        print("\nCtrl+C entered.\nExiting...")