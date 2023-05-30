#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs
import argparse
import tf

from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Float32MultiArray, String
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from utils import MoveHandler
import math
import numpy as np
import time


class Movement(object):

	def __init__(self):

		super(Movement, self).__init__()

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
		
		try:
			self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
			if self.is_gripper_present:
				gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
				self.gripper_joint_name = gripper_joint_names[0]
			else:
				self.gripper_joint_name = ""
			self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

			# Create the MoveItInterface necessary objects
			# self.setup_moveit()
			# self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
			rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
		
		except Exception as e:
			print (e)
			self.is_init_success = False
		else:
			self.is_init_success = True


	def reach(self):
		joint_positions = self.arm.get_current_joint_values()
		joint_positions[0] = 1.4942272496618103
		joint_positions[1] = -1.5880536106586103
		joint_positions[2] = -0.03401282817805562
		joint_positions[3] = 0.1416210478160157
		joint_positions[4] = -0.006562028925954699
		joint_positions[5] = -0.16848222320277273
		
		self.arm.set_goal_joint_tolerance(.1)

		self.arm.set_joint_value_target(joint_positions)
		self.arm.set_path_constraints(None)
		self.arm.go(wait=False)

	def hunting(self):
		joint_positions = self.arm.get_current_joint_values()

		joint_positions[0] = 1.7792723053518253
		joint_positions[1] = 0.9441798223296075
		joint_positions[2] = -1.6463384890116481   
		joint_positions[3] = -1.668023011871509
		joint_positions[4] = -1.9912231764990898
		joint_positions[5] = -1.4057900614468952
		self.arm.set_goal_joint_tolerance(.1)

		self.arm.set_joint_value_target(joint_positions)
		self.arm.set_path_constraints(None)
		self.arm.go(wait=False)

	def reset(self):
		joint_positions = self.arm.get_current_joint_values()

		joint_positions[0] = 0.15
		joint_positions[1] = 0.47
		joint_positions[2] = 2.61   
		joint_positions[3] = 0
		joint_positions[4] = 0
		joint_positions[5] = -1.57
		self.arm.set_goal_joint_tolerance(.1)

		self.arm.set_joint_value_target(joint_positions)
		self.arm.set_path_constraints(None)
		self.arm.go(wait=False)

	def set_grip(self, position):
		gripper_joint = self.robot.get_joint(self.gripper_joint_name)
		gripper_joint.move(position)

	def open(self):
		self.set_grip(0.9)

	def close(self):
		self.set_grip(0.01)
	