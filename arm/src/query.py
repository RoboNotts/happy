#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs
import argparse
import tf

from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray, String, Empty
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from utils import MoveHandler
import math
import numpy as np
import time


class QueryCartesian(object):

	def __init__(self):

		super(QueryCartesian, self).__init__()
		rospy.init_node('happy_arm_query')


		moveit_commander.roscpp_initialize(sys.argv)
		# rospy.init_node('movement', anonymous=True)

		self.robot = moveit_commander.RobotCommander('/gen3lite/robot_description')
		self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

		rospy.loginfo(rospy.get_namespace())
		print(self.robot.get_group_names())

		group_name = 'arm' 
		self.arm = moveit_commander.MoveGroupCommander(group_name, ns=rospy.get_namespace())
		self.arm.set_planner_id('TRRT')
		
		grip_name = 'gripper'
		self.gripper = moveit_commander.MoveGroupCommander(grip_name, ns=rospy.get_namespace())

		self.planning_frame = self.arm.get_planning_frame()
		self.eef_link = self.arm.get_end_effector_link()
		self.group_names = self.robot.get_group_names()

		self.move_handler = MoveHandler(self.arm)
		self.grip_handler = MoveHandler(self.gripper)
		
		moveit_commander.roscpp_initialize(sys.argv)

		try:
			self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
			if self.is_gripper_present:
				rospy.loginfo('Gripper found')
				gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
				self.gripper_joint_name = gripper_joint_names[0]
			else:
				rospy.loginfo("Gripper not found")
				self.gripper_joint_name = ""
			self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

		except Exception as e:
			print (e)
			self.is_init_success = False
		else:
			self.is_init_success = True


		rospy.Subscriber("/arm/query/cartesian", Empty, self.cartesian_query)
		rospy.spin()
	
	def cartesian_query(self, msg):
		pose = self.arm.get_current_pose()
		rospy.loginfo("Actual cartesian pose is : ")
		rospy.loginfo(pose.pose)


	
if __name__ == '__main__':
	query_cartesian = QueryCartesian()

