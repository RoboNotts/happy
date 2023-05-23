#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import tf
from moveit_commander.conversions import pose_to_list
import math
from utils import WorldConfig

class PlanningSceneConfigurator(object):

	def __init__(self):

		super(PlanningSceneConfigurator, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('happy_planning_scene_configurator', anonymous=True)

		self.robot = moveit_commander.RobotCommander('/gen3lite/robot_description')
		self.setup_scene()

	def clear_scene(self):
		self.scene.remove_world_object()

	def scene_add_object(self, object_name, x, y, z, dimensions):

		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = x
		p.pose.position.y = y
		p.pose.position.z = z

		self.scene.add_box(object_name, p, dimensions)

	def setup_scene(self):
		self.scene = moveit_commander.PlanningSceneInterface()

		# Ensure everything comes up before adding to the planning scene
		rospy.sleep(2)

		self.clear_scene()
		self.scene_add_object("floor", 0, 0, -0.63, (8, 8, .01)) 
		world_config = WorldConfig()
		for wobj in world_config.get_objects():
			self.scene_add_object(wobj["name"], wobj["x"], wobj["y"], wobj["z"], wobj["dims"])
		
	def get_target_coords(self):
		return x, y, z

if __name__ == '__main__':

	print("**** PLANNING SCENE CONFIGURATOR STARTING ****")

	planningSceneConfigurator = PlanningSceneConfigurator()
