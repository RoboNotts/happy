#!/usr/bin/env python3

import rospy
import numpy as np
import math
# from feather_pointer import *
# from predefined_poses import *
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point
from arm.srv import Grasp, GraspResponse, SimplePose, SimplePoseResponse 

# from catroyale_mgt.msg import CommandTask, CatLocation, Task
from moveit_msgs.msg import MoveGroupActionFeedback
from utils import MoveHandler
from movements import Movement
from time import sleep

class CommandHandler(object):

	def __init__(self):

		super(CommandHandler, self).__init__()

		rospy.init_node('happy_arm_command')

		self.movement = Movement()
		
		rospy.Subscriber("/arm/command/position", String, self.process_command)
		rospy.Subscriber("/arm/command/cartesian", Point, self.process_point)
		# rospy.Subscriber("/arm/command/pick", Point, self.process_pick)
		self.grasp_service = rospy.Service('/arm/command/grasp', Grasp, self.process_grasp)
		self.pose_service = rospy.Service('/arm/command/simplePose', SimplePose, self.process_point)

		rospy.spin()

	def process_grasp(self, msg):
		rospy.loginfo(msg)
		point = Point()
		point.x = msg.x
		point.y = msg.y
		point.z = msg.z
		
		self.movement.open()
		sleep(5)
		self.movement.poised()
		sleep(20)
		self.movement.poised2()
		sleep(20)
		self.movement.open()
		sleep(5)
		self.movement.move_to_point(msg)
		sleep(15)
		self.movement.close()
		sleep(5)
		self.movement.lift()
		sleep(10)
		return GraspResponse(True)

	def euler_to_quarternion(r,p,y):
		qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
		qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
		qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
		qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
		
		return [qx,qy,qz,qw]
		
	def flatQ(self):
		lx = -math.pi / 2
		ly = math.pi
		lz = 0

		return self.euler_to_quarternion(lx, ly, lz)

	def process_point(self, msg):
		#if msg.effectorP == "Flat":
		# lx = -math.pi / 2
		# ly = math.pi
		# lz = 0
		
		# qx, qy, qz, qw = self.euler_to_quarternion(lx, ly, lz)

		# self.movement.move_to_pose(msg.x, msg.y, msg.z, qx, qy, qz, qw)
		self.movement.move_to_point((msg.x, msg.y, msg.z))

	def process_command(self, msg):
		rospy.loginfo(msg.data)
		
		if msg.data == 'reach_low':
			self.movement.reach_low()
			
		if msg.data == 'reach_high':
			self.movement.reach_high()
			
		if msg.data == 'hunting':
			self.movement.hunting()

		if msg.data == 'home':
			self.movement.reset()
			
		if msg.data == 'open':
			self.movement.open()
			
		if msg.data == 'close':
			self.movement.close()
			
		if msg.data == 'test_cart':
			self.movement.cartesian_test()

		if msg.data == 'poised':
			self.movement.poised()
			
if __name__ == '__main__':
	cmd_handler = CommandHandler()

	#cmd_handler.get_current_pose()