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
import copy


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
		
		print("==== PRINTING ROBOT STATE")
		print(self.robot.get_current_state())
		print("")


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
	
	# Moves arm by a specified delta for each position
	# Keeps orientation the same
	def move_point_diff(self, dx, dy, dz):
		print("==== CURRENT POSE")
		current_pose = self.arm.get_current_pose().pose
		print(current_pose)

		pose_goal = current_pose
		pose_goal.position.x += dx
		pose_goal.position.y += dy
		pose_goal.position.z += dz

		self.arm.set_pose_target(pose_goal)
		succ = self.arm.go(wait=True)
		self.arm.stop()
		self.arm.clear_pose_targets()

		return succ
	

	def move_to_pose(self, lx, ly, lz, x, y, z, w):
		print("==== CURRENT POSE")
		current_pose = self.arm.get_current_pose().pose
		print(current_pose)
		
		pose_goal = current_pose
		pose_goal.orientation.x = x
		pose_goal.orientation.y = y
		pose_goal.orientation.z = z
		pose_goal.orientation.w = w
		pose_goal.position.x = lx
		pose_goal.position.y = ly
		pose_goal.position.z = lz
		
		self.arm.set_pose_target(pose_goal)
		succ = self.arm.go(wait=True)
		self.arm.stop()
		self.arm.clear_pose_targets()

		return succ
	
	def poised(self):
		joint_positions = [1.4392047436441193, -0.1685397474823187, 2.4947236079827784, -1.7980481236691022, 1.096844999186182, 1.6095996491417872]
		self.move_to_joint(joint_positions)

	def poised2(self):
		joint_positions = [2.4147382278033387, -0.8096169503506285, 1.5959259148408842, -0.5561937368119967, 1.1516504575750315, 1.0981803081567476]
		self.move_to_joint(joint_positions)

	def lift(self):
		joint_positions = [2.1169275757441377, -0.8745852977753303, 1.0230999380730157, -0.43398447018156894, 0.6362366412709822, 0.7577060817505864]
		self.move_to_joint(joint_positions)

	def reach_low(self):
		joint_positions = [1.4942272496618103, -1.5880536106586103, -0.03401282817805562, 0.1416210478160157, -0.006562028925954699, -0.16848222320277273]
		self.move_to_joint(joint_positions)	
		
	def reach_high(self):
		joint_positions = [ 1.4037335684531358, -1.2520926433518067, -0.0748369571600982, 0.2086179250430301, -0.05491651220630622, -0.17301705390696043]
		self.move_to_joint(joint_positions)	
		
	def hunting(self):
		joint_positions = [1.7198001897848298, 0.38789394014389417, -2.3331043509091396, -1.5667152987404593, -2.445029554674118, -1.3058512131161457]
		self.move_to_joint(joint_positions)	

	def reset(self):
		joint_positions = [0.15, 0.47, 2.61, 0, 0, -1.57]
		self.move_to_joint(joint_positions)	

	def move_to_joint(self, joint_positions):
		self.arm.set_goal_joint_tolerance(.1)
		self.arm.set_joint_value_target(joint_positions)
		self.arm.set_path_constraints(None)
		self.arm.go(wait=False)

	def set_grip(self, position):
		gripper_joint = self.robot.get_joint(self.gripper_joint_name)
		gripper_joint.move(position)

	def move_to_pose2(self, pose_goal):
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
		
		actual_pose.position.x, actual_pose.position.y, actual_pose.position.z = point
		
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
	