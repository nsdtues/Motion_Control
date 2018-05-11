#!/usr/bin/env python

import rospy
from motion_control.msg import sys_cmd_msg_to_motor

class GET_PARA(object):
    def __init__(self):
		rospy.init_node('get_para_node')
		self.pub = rospy.Publisher('sys_cmd_msg_to_motor',sys_cmd_msg_to_motor,queue_size=1)
		
		rate = rospy.Rate(50) # 50Hz
		while not rospy.is_shutdown():
		
			cmd_msg = sys_cmd_msg_to_motor();
			cmd_msg.syscmd = 'cmdmotorinitial'
			self.pub.publish(cmd_msg)

			rate.sleep()

			
if __name__ == '__main__':
    GET_PARA()

