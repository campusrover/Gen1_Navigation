#!/usr/bin/env python
import rospy
import tf
from tf import TransformerROS, Transformer 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseWithCovariance, Pose, Point, Quaternion
from apriltags2_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Header
import numpy as np

rospy.init_node('tag_localization')

transformer = TransformerROS()
listener = tf.TransformListener()
robot_covariance = None

def amcl_pose_callback(msg):
	robot_covariance = msg.pose.covariance

def tag_detection_callback(msg):
	if msg.detections:
		robot_to_tag_pose = PoseStamped(msg.detections[0].pose.header, msg.detections[0].pose.pose.pose)	
		
		print robot_to_tag_pose
		
		x = robot_to_tag_pose.pose.position.x + .585 ##bring in tag attribute here 
		y = 7.85 - robot_to_tag_pose.pose.position.y
		z = robot_to_tag_pose.pose.position.z

		#### ANGLE CALCULATION ####
		
		first_angle = np.arctan(.585/7.85)
		second_angle = np.arctan(y/x)
		
		middle_angle = 90 - (first_angle + second_angle)
		print middle_angle

		robot_to_map_angle = float(first_angle + middle_angle)
		##print robot_to_map_angle.__class__.__name__
		
		robot_quaternion = quaternion_from_euler(0, 0, robot_to_map_angle)
		robot_quaternion = robot_quaternion.tolist()
		robot_quaternion =  Quaternion(robot_quaternion[0], robot_quaternion[1], robot_quaternion[2], robot_quaternion[3])

		#print robot_to_tag_pose.pose.orientation.__class__.__name__
		
		############################

		robot_point = Point(x, y, z)
		robot_pose = Pose(robot_point, robot_quaternion)
		robot_covariance = [0] * 36
		header = Header()
		header.stamp = rospy.Time.now()
		
		robot_position = PoseWithCovariance(robot_pose, robot_covariance)
		robot_position_pose = PoseWithCovarianceStamped(header, robot_position)
		pose_estimate_pub.publish(robot_position_pose)
		rate.sleep()


tag_detection_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detection_callback)
amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
pose_estimate_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
br = tf.TransformBroadcaster()
rate = rospy.Rate(50000)

while not rospy.is_shutdown():
	br.sendTransform(
		(0.585, 7.85, -0.00143),
		(0.716, -0.004, 0.002, 0.696),
		rospy.Time.now(),
		'tag_6',
		'map'
	)
	rate.sleep()
