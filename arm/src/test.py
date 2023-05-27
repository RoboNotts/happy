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


class Move2Pose(object):

	def __init__(self):
		super(Move2Pose, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move2pose', anonymous=True)

		self.robot = moveit_commander.RobotCommander('/gen3lite/robot_description')
		self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

		rospy.loginfo(rospy.get_namespace())

		print(self.robot.get_group_names())

		group_name = 'arm' #'manipulator for ur3, {arm, gripper} for gen3lite'
		self.arm = moveit_commander.MoveGroupCommander(group_name, ns=rospy.get_namespace())
		# self.arm.set_goal_position_tolerance(0.01)
		# self.arm.set_goal_orientation_tolerance(0.1)
		self.arm.set_planner_id('TRRT')
		# self.arm.set_goal_joint_tolerance(0.001)


		grip_name = 'gripper'
		self.gripper = moveit_commander.MoveGroupCommander(grip_name, ns=rospy.get_namespace())

		# gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
		# self.gripper_joint_name = gripper_joint_names[0]

		# # self.gripper.set_goal_position_tolerance(0.01)
		# # self.gripper.set_goal_orientation_tolerance(0.1)
		# # self.gripper.set_planner_id('TRRT')

		# self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		# 											   moveit_msgs.msg.DisplayTrajectory,
		# 											   queue_size=20)

		self.planning_frame = self.arm.get_planning_frame()
		print("============ Planning frame: %s" % self.planning_frame)

		self.eef_link = self.arm.get_end_effector_link()
		print("============ End effector link: %s" % self.eef_link)

		self.group_names = self.robot.get_group_names()
		print("============ Available Planning Groups:", self.robot.get_group_names())

		self.move_handler = MoveHandler(self.arm)
		self.grip_handler = MoveHandler(self.gripper)
		
		moveit_commander.roscpp_initialize(sys.argv)
		# rospy.init_node('command_handler')

		try:
			self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
			if self.is_gripper_present:
				gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
				self.gripper_joint_name = gripper_joint_names[0]
			else:
				self.gripper_joint_name = ""
			self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

			# Create the MoveItInterface necessary objects
			self.setup_moveit()
			self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
																										moveit_msgs.msg.DisplayTrajectory,
																										queue_size=20)

			# if self.is_gripper_present:
			# 	gripper_group_name = "gripper"
			# 	self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

			rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
		except Exception as e:
			print (e)
			self.is_init_success = False
		else:
			self.is_init_success = True

	def setup_moveit(self):
		arm_group_name = "arm"
		self.robot = moveit_commander.RobotCommander("robot_description")
		self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
		self.arm = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
		# self.grip = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
			
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
		
		# self.grip(0.25)

	def open(self):
		self.set_grip(0.9)

	def close(self):
		self.set_grip(0.1)
	
	def zero(self):
		joint_positions = self.arm.get_current_joint_values()

		joint_positions[0] = 0
		joint_positions[1] = 0
		joint_positions[2] = 0   
		joint_positions[3] = 0
		joint_positions[4] = 0
		joint_positions[5] = 0
		
		self.arm.set_joint_value_target(joint_positions)
		self.arm.set_path_constraints(None)
		self.arm.go(wait=False)

	def halt(self):
		self.arm.stop()

	def go_joint_space(self, joints, wait=False):
		rospy.loginfo("Go Joint Space")
		# self.arm.set_goal_joint_tolerance(.1)
		self.setup_moveit()
		self.arm.go(joints[:6], wait=wait)

	def follow_trajectory(self, joint_names, joints, velocities, efforts, accelerations, times, time_scale = 1): 
		rospy.loginfo("Starting Trajectory")
		
		times = times * time_scale

		np_times = np.array(times)
		np_times = np_times.reshape(len(times), 1)
		
		np_joints = np.array(joints)
		np_joints = np_joints[:, :6]

		dt = np.tile(np.diff(np_times, axis=0),(1,6))
		velocities = np.divide(np.diff(np_joints, axis=0), dt)
		velocities = np.concatenate((velocities, np.zeros((1,6))), axis=0)
		accelerations = np.diff(velocities, axis=0)/dt
		accelerations = np.concatenate((accelerations, np.zeros((1,6))), axis=0)

		trajectory = RobotTrajectory()
		trajectory.joint_trajectory.header.frame_id = "base_link"
		trajectory.joint_trajectory.joint_names = joint_names[:6]
		trajectory.joint_trajectory.points = []
		for waypoint in range(0, np.shape(np_joints)[0]):
			j_point = JointTrajectoryPoint()
			j_point.positions = np_joints[waypoint, :]
			j_point.velocities = velocities[waypoint, :]
			j_point.accelerations = accelerations[waypoint, :]
			j_point.time_from_start = rospy.Duration.from_sec(np_times[waypoint,0])
			j_point.time_from_start = rospy.Time(np_times[waypoint,0])
			trajectory.joint_trajectory.points.append(j_point)

		self.setup_moveit()
		self.arm.execute(trajectory, wait=False)

	def test_traj(self): # Can delete once working
		print("TEST_TRAJ")
		rospy.loginfo("TEST_TRAJ")

		self.setup_moveit()
		arm_group = self.arm

		rospy.loginfo("going to starting position")
		joints = [0.14326741400190268, -0.14454200289961427, 2.210325700437653, -2.5811096295261144, 1.939946140238486, -0.17301279284921645]
		arm_group.go(joints, wait=True)	

		trajectory = RobotTrajectory()
		trajectory.joint_trajectory.header.frame_id = "base_link"
		trajectory.joint_trajectory.joint_names = [
			'joint_1',
			'joint_2',
			'joint_3',
			'joint_4',
			'joint_5',
			'joint_6'
		]
		trajectory.joint_trajectory.points = []

		j_point = JointTrajectoryPoint()
		j_point.positions = [0.14325715251489968, -0.14441256344306366, 2.2102757288569537, -2.5809500433148744, 1.9397525149416734, -0.1730436913150741]
		j_point.velocities = [-0.000724363929957437, 0.009137201402789355, -0.0035275209695156753, 0.011265277157879783, -0.01366811465992214, -0.002181139453936675]
		j_point.accelerations = [-0.03408882831307224, 0.42999999999995475, -0.1660064115945773, 0.5301480140744854, -0.6432264152534142, -0.10264521091834689]
		j_point.time_from_start = rospy.Time(0, 42498611)
		trajectory.joint_trajectory.points.append(j_point)

		j_point = JointTrajectoryPoint()
		j_point.positions = [0.14048669666144897, -0.10946574725909933, 2.196784111295665, -2.537864031226138, 1.8874764328520581, -0.18138583856653473]
		j_point.velocities = [-0.013761772378056876, 0.17359241767469155, -0.06701733567022439, 0.2140225011363596, -0.25967262450251427, -0.0414382100605727]
		j_point.accelerations = [-0.034082855353790596, 0.4299246564748502, -0.1659773243660313, 0.5300551228646786, -0.6431137107288782, -0.10262722568107738]
		j_point.time_from_start = rospy.Time(0, 424986112)
		trajectory.joint_trajectory.points.append(j_point)

		j_point = JointTrajectoryPoint()
		j_point.positions = [0.13770597932099524, -0.07438949161858438, 2.1832425221536775, -2.494618432926162, 1.8350067254656302, -0.18975888428385304]
		j_point.velocities = [-0.01946597393136392, 0.24554580502479448, -0.09479576273083762, 0.3027340020889032, -0.367305925457291, -0.05861418824857645]
		j_point.accelerations = [-0.034078837555674375, 0.42987397555457096, -0.1659577584179504, 0.5299926382386015, -0.643037898387765, -0.10261512764909804]
		j_point.time_from_start = rospy.Time(0, 592358761)
		trajectory.joint_trajectory.points.append(j_point)

		try:
			arm_group.execute(trajectory, wait=True)
			return True
		except:
			return False
		


#------------------------------------------------------------------#


	def reach_cartesian_pose(self, pose, tolerance, constraints):
		arm_group = self.arm
		arm_group.set_goal_position_tolerance(tolerance)

		if constraints is not None:
			arm_group.set_path_constraints(constraints)

		arm_group.set_pose_target(pose)

		# Plan and execute
		rospy.loginfo("Planning and going to the Cartesian Pose")
		return arm_group.go(wait=True)

	def _parse_interactive_params(self, params):
		try:
			parts = params.split(" ")
			x = float(parts[0])
			y = float(parts[1])
			if len(parts) > 2:
				z = float(parts[2])
			else:
				z = 0.4
		except ValueError:
			return None, None, None, "Incorrect syntax detected. Please provide x y z coordinates." 

		return self._coords_to_twist(x, y, z), None


	def moverandom(self):

		lx = -0.055762689160819695
		ly = 0.5497687748697093
		lz = 0.18957475433301452

		x =  -0.6451481562452683
		y = -0.6844028908489487
		z =  -0.24755764495552413
		w =  0.23257633567284594

		self.move_to_pose(lx, ly, lz, x, y, z, w)

	def move_pre_ee(self):
		lx = -0.09382775312988186
		ly = -0.3847031552088921
		lz = 0.4523195907252573

		x = 0.7149280401082445
		y = -0.06533840395957102
		z = -0.018613261774884267
		w = 0.695889601101425

		self.move_to_pose(lx, ly, lz, x, y, z, w)

	def move_to_ee(self):
		lx = -0.14941607919723288
		ly = -0.6653192099040665
		lz = 0.4513665226745428

		x = 0.7066147118507385
		y = -0.0580654388302422
		z = -0.026108068001366284
		w = 0.7047286162735168

		self.move_to_pose(lx, ly, lz, x, y, z, w)

	def move_lift_ee(self):
		lx = -0.1470849771825869
		ly = -0.6578664705942637
		lz = 0.5273860375015894

		x = 0.706661815160615
		y = -0.05789379292961242
		z = -0.026485298959035713
		w = 0.704681429209809
   
		self.move_to_pose(lx, ly, lz, x, y, z, w)

	def grasp_ee(self):
		self.set_grip(0.9)
		
	def release_ee(self):
		self.set_grip(0.5)
		
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

	# def set_grip(self, position):
	# 	gripper_joint = self.robot.get_joint(self.gripper_joint_name)
	# 	rospy.loginfo(gripper_joint)
	# 	# gripper_joint.move(position)

	def set_grip(self, position):
		gripper_joint = self.robot.get_joint(self.gripper_joint_name)
		gripper_joint.move(position)

	def get_current_pose(self):
		rospy.loginfo('Getting current pose')
		current_pose = self.arm.get_current_pose()
		rospy.loginfo(current_pose)


if __name__ == '__main__':
	move2pose = Move2Pose()
	move2pose.get_current_pose()

	# move2pose.moverandom()
	# move2pose.reset()
	rospy.loginfo('-----------------')
	move2pose.reach()

	time.sleep(5)
	rospy.loginfo('-----------------')
	move2pose.open()
	time.sleep(5)
	rospy.loginfo('-----------------')
	move2pose.close()
	
	# rospy.loginfo('-----------------')
	# move2pose.close()
	# rospy.loginfo('-----------------')
	# move2pose.open()
	# rospy.loginfo('-----------------')
	# move2pose.close()
	# rospy.loginfo('-----------------')
	

	# move2pose.moverandom()
	# rospy.loginfo('-----------------')
	# move2pose.release_ee()
	# rospy.loginfo('-----------------')
	# move2pose.move_pre_ee()
	# rospy.loginfo('-----------------')
	# move2pose.move_to_ee()
	# rospy.loginfo('-----------------')
	# move2pose.grasp_ee()
	# rospy.loginfo('-----------------')
	# move2pose.move_lift_ee()
	# rospy.loginfo('-----------------')
	# move2pose.moverandom()
