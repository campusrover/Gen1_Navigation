#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [(0.724, 0.276, 0.00638), (0.0, 0.0, 0.0, 1.0)], [(0,0,0), (0.0, 0.0, 0.0, 0.0)] 
##list of waypoints to control; just a relative coordinate and quaternion. These waypoints are currently just hard coded in

def goal_pose(pose):
	set_goal = MoveBaseGoal()
	set_goal.target_pose.header.frame_id = 'map'
	set_goal.target_pose.pose.position.x = pose[0][0]
	set_goal.target_pose.pose.position.y = pose[0][1]
	set_goal.target_pose.pose.position.z = pose[0][2]
	set_goal.target_pose.pose.orientation.x = pose[1][0]
	set_goal.target_pose.pose.orientation.y = pose[1][1]
	set_goal.target_pose.pose.orientation.z = pose[1][2]
	set_goal.target_pose.pose.orientation.w = pose[1][3]

	return set_goal

if __name__ == '__main__':
	rospy.init_node('navigate')
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	while True:
		for pose in waypoints:
			goal = goal_pose(pose)
			client.send_goal(goal)
			client.wait_for_result()


