#!/usr/bin/env python

import rospy
import os
import sys
import datetime
import zmq


class FIXED_GAIT_PUB(object):
    def __init__(self):
		rospy.init_node('fixed_gait_pub')
		
		self.cur_path = os.path.abspath(os.path.dirname(os.path.abspath(sys.argv[0])))	
		self.pub_log_file_path = self.cur_path + "/../log/" + "pubdata " + datetime.datetime.now().strftime('%Y-%m-%d-%H:%M:%S') + ".txt"
		
		
		# bind to gait stream
		self.fixed_gait_stream = "tcp://*:8011"
		self.fixed_gait_context = zmq.Context()
		self.fixed_gait_socket = self.fixed_gait_context.socket(zmq.PUB)
		self.fixed_gait_socket.bind(self.fixed_gait_stream)

		rate = rospy.Rate(67) # 67Hz
		with open(self.pub_log_file_path, "w") as self.GaitPubFile:
		    print(self.pub_log_file_path)
		    # self.GaitPubFile.writelines(self.pub_log_file_path + '\n')
		    self.gait_cnt = 0
		    while not rospy.is_shutdown():
			    self.gait_cnt = self.gait_cnt + 1
			    if self.gait_cnt < 27:
			        self.gait = "GaitL:A, GaitR:B, Gait:GaitWalking"
			    elif self.gait_cnt < 60:
			    	self.gait = "GaitL:B, GaitR:B, Gait:GaitWalking"
			    elif self.gait_cnt < 100:
			    	self.gait = "GaitL:C, GaitR:B, Gait:GaitWalking"
			    else :
			    	self.gait_cnt = 0
			    
			    self.fixed_gait_socket.send_string(self.gait)		
			    self.now = datetime.datetime.now()
			    nwrite = "{} ".format(self.now) + self.gait + "\n"
			    self.GaitPubFile.writelines(nwrite)
			    print("pub {}, ".format(self.now) + self.gait)	
			    rate.sleep()


if __name__ == '__main__':
    try:
        FIXED_GAIT_PUB()
    except KeyboardInterrupt:
        print("\nCtrl+C entered.\nExiting...")
