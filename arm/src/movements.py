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

			# Create the MoveItInterface necessary objects
			# self.setup_moveit()
			# self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
			# rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
		
		except Exception as e:
			print (e)
			self.is_init_success = False
		else:
			self.is_init_success = True

	# def setup_moveit(self):
	# 	arm_group_name = "arm"
	# 	self.robot = moveit_commander.RobotCommander("robot_description")
	# 	self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
	# 	self.arm = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
	# 	# self.grip = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
			

	def move_to_pose(self, lx, ly, lz, x, y, z, w):
		pose_goal = Pose()
		pose_goal.orientation.x = x
		pose_goal.orientation.y = y
		pose_goal.orientation.z = z
		pose_goal.orientation.w = w
		pose_goal.position.x = lx
		pose_goal.position.y = ly
		pose_goal.position.z = lz

		result = self.move_handler.request_movement(pose_goal)
		return result

	def reach_low(self):
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

	def reach_high(self):
		joint_positions = self.arm.get_current_joint_values()
		joint_positions[0] = 1.4037335684531358
		joint_positions[1] = -1.2520926433518067
		joint_positions[2] = -0.0748369571600982
		joint_positions[3] = 0.20861792504303012
		joint_positions[4] = -0.05491651220630622
		joint_positions[5] = -0.17301705390696043
		
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

	def move_to_pose(self, pose_goal):
		pose = self.arm.get_current_pose()
		rospy.loginfo("Actual cartesian pose is : ")
		rospy.loginfo(pose.pose)

		rospy.loginfo(f"received data {pose_goal}")
		# pose_goal = Pose()
		# pose_goal.orientation.x = x
		# pose_goal.orientation.y = y
		# pose_goal.orientation.z = z
		# pose_goal.orientation.w = w
		# pose_goal.position.x = lx
		# pose_goal.position.y = ly
		# pose_goal.position.z = lz

		result = self.move_handler.request_movement(pose_goal)
		return result

	def move_to_point(self, point):
		pose_stamped = self.arm.get_current_pose()
		actual_pose = pose_stamped.pose
		rospy.loginfo("Actual cartesian pose is : ")
		rospy.loginfo(actual_pose)

		rospy.loginfo(f"received point {point}")
		
		actual_pose.position = point
		
		result = self.move_handler.request_movement(actual_pose)
		return result

	def cartesian_test(self):
		pose = self.arm.get_current_pose()
		rospy.loginfo("Actual cartesian pose is : ")
		rospy.loginfo(pose.pose)

		actual_pose = pose.pose

		actual_pose.position.y -= 0.2
		success = False
		success &= self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
		print(success)

		pose = self.arm.get_current_pose()
		rospy.loginfo("New cartesian pose is : ")
		rospy.loginfo(pose.pose)


	def reach_cartesian_pose(self, pose, tolerance, constraints):
		arm_group = self.arm
		arm_group.set_goal_position_tolerance(tolerance)

		if constraints is not None:
			arm_group.set_path_constraints(constraints)

		arm_group.set_pose_target(pose)

		# Plan and execute
		rospy.loginfo("Planning and going to the Cartesian Pose")
		return arm_group.go(wait=True)


	def open(self):
		self.set_grip(0.9)

	def close(self):
		self.set_grip(0.1)
	