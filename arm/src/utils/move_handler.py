#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
import math

class MoveHandler(object):

    def __init__(self, move_group):
        super(MoveHandler, self).__init__()

        self.move_group = move_group
        
        rospy.Subscriber('/move_group/status', GoalStatusArray, self.callback_movement_status)
        rospy.Subscriber('/joint_states', JointState, self.callback_check_joint_states)

        self.goal_statuses = {}
        self.goal_positions = {}

        self.current_goal_id = None
        self.current_position = None

        self.monitor_joint_states = False
        self.joint_limits_breached = False
        self.joint_stack = []

        self.ROBOT_STATE_CATCHUP_TIME = 1

        self.joint_file = 'joints_state.txt'
        # self.RECORD_STATES = false
        
    def callback_check_joint_states(self, joint_states_msg):
        if self.monitor_joint_states:
            joint_file = open(self.joint_file, 'a')

            if len(self.joint_stack) == 0:
                self.joint_stack.append(joint_states_msg.position)

            last_joint_positions = self.joint_stack[-1]
            matching = True
        
            for i, joint_position in enumerate(joint_states_msg.position):
                if round(joint_position, 4) != round(last_joint_positions[i], 4):
                    matching = False
                
                if joint_position > math.pi or joint_position < -math.pi:
                    self.joint_limits_breached = True
            
            if not matching:
                joint_file.write("{}".format(joint_states_msg.position))
                joint_file.write("\n")

                self.joint_stack.append(joint_states_msg.position)
            
            if self.joint_limits_breached:
                joint_file.write('JOINTS BREACHED LIMITS')
                joint_file.write("\n")

            joint_file.close()

    def callback_movement_status(self, goal_status_array):

        # uint8 PENDING=0
        # uint8 ACTIVE=1
        # uint8 PREEMPTED=2
        # uint8 SUCCEEDED=3
        # uint8 ABORTED=4
        # uint8 REJECTED=5
        # uint8 PREEMPTING=6
        # uint8 RECALLING=7
        # uint8 RECALLED=8
        # uint8 LOST=9
        
        for item in goal_status_array.status_list:

            if not item.goal_id.id in self.goal_statuses:
                self.current_goal_id = item.goal_id.id
                self.goal_statuses[item.goal_id.id] = item.status
                self.goal_positions[item.goal_id.id] = self.current_position

            elif self.goal_statuses[item.goal_id.id] != item.status:
                self.goal_statuses[item.goal_id.id] = item.status

                if item.status != 3:
                    print("Item status update:", item.status)


    def _get_current_status(self):
        if self.current_goal_id in self.goal_statuses:
            return self.goal_statuses[self.current_goal_id]
        else:
            print("Trying to retrieve non-existant status for ", self.current_goal_id)
            return None
            
    def _get_pose_resolution(self):
        while self._get_current_status() == 1:
            print(".", end="")
            rospy.sleep(1)
            
        return self._get_current_status() 

    def request_movement(self, pose): 
        joint_file = open(self.joint_file, 'a')
        joint_file.write("Pose Requested:\n")
        joint_file.write(str(pose))
        joint_file.close()

        self.current_goal_id = None
        self.current_position = (pose.position.x, pose.position.y, pose.position.z)

        self.move_group.set_pose_target(pose)
        plan = self.move_group.go(wait=True)
        
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if self._get_pose_resolution() == 3:
            return True
        else:
            print("Move Failed: ", self.current_goal_id, self.current_position, self._get_current_status())
            return False

    def reset_state(self, state):
        # used as a fix for a race condition causing the starting state tolerance check to fail
        # self.move_group.set_start_state(state)
        # rospy.sleep(self.ROBOT_STATE_CATCHUP_TIME)
        pass

    def reset_arm(self):
        print("Resetting arm...............")
        joint_goal = self.move_group.get_current_joint_values()

        joint_goal[0] = -0.029380824486064938
        joint_goal[1] = -1.6124287307187872
        joint_goal[2] = 2.1725598372236163
        joint_goal[3] = 2.581307739190928
        joint_goal[4] =-1.540456272053996
        joint_goal[5] = -0.00026977155912177864

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        current_pose = self.move_group.get_current_pose().pose
        current_joints = self.move_group.get_current_joint_values()

    def get_current_pose(self):
        return self.move_group.get_current_pose()

if __name__ == '__main__':
    mh = MoveHandler(None)
