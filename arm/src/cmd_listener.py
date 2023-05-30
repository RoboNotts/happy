#!/usr/bin/env python3

import rospy
# from feather_pointer import *
# from predefined_poses import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# from catroyale_mgt.msg import CommandTask, CatLocation, Task
from moveit_msgs.msg import MoveGroupActionFeedback
from utils import MoveHandler
from movements import Movement


class CommandHandler(object):

	def __init__(self):

		super(CommandHandler, self).__init__()

		rospy.init_node('happy_arm_command')

		self.movement = Movement()
		# self.service = rospy.ServiceProxy('speak', Speak)

		rospy.Subscriber("/arm/command/position", String, self.process_command)
		rospy.Subscriber("/arm/command/pose", Pose, self.process_pose)
		rospy.spin()


	def process_pose(self, msg):
		rospy.loginfo(msg)


	def process_command(self, msg):
		rospy.loginfo(msg.data)
		
		if msg.data == 'reach_low':
			# self.service("Arm is about to move. Please stand clear.")
			self.movement.reach_low()
			pass

		if msg.data == 'reach_high':
			# self.service("Arm is about to move. Please stand clear.")
			self.movement.reach_high()
			pass

		if msg.data == 'hunting':
			# self.service("Arm is about to move. Please stand clear.")
			self.movement.hunting()

		if msg.data == 'home':
			# self.service("Arm is about to move. Please stand clear.")
			self.movement.reset()
			pass

		if msg.data == 'open':
			self.movement.open()
			pass

		if msg.data == 'close':
			self.movement.close()
			pass



if __name__ == '__main__':
	cmd_handler = CommandHandler()

	# cmd_handler.get_current_pose()