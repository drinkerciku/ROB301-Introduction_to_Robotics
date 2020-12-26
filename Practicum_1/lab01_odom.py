#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry

def get_yaw_from_quarternion(q):
	siny_cosp = 2*(q.w*q.z + q.x*q.y)
	cosy_cosp = 1 - 2*(q.y*q.y+q.z*q.z)
	yaw = math.atan(siny_cosp/cosy_cosp)
	return yaw


def callback(odom_data):
	'''TODO: complete the call back function for subscriber'''
	
	point = odom_data.pose.pose.position
	quart = odom_data.pose.pose.orientation
	theta = get_yaw_from_quarternion(quart)
	cur_pose = (point.x, point.y, theta)

	rospy.loginfo(cur_pose)

	pass


def main():
	try:
		rospy.init_node('odometry')
		'''TODO: initialize the subscriber of odometery here'''
		odom_subscriber = rospy.Subscriber('odom', Odometry, callback, queue_size = 1)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__ == '__main__':
	main()