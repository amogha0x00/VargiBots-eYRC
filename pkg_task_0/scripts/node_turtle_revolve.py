#!/usr/bin/env python

"""

Team id : 1823
Theme :  Vargi Bots (VB)
Author : Amoghavarsha S G

"""

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

PI = 3.14159265
TWO_PI = 2 * PI
ONCE = 1
RADIANS_COVERED = 0
ROTATION_COUNT = 0
PREV_THETA = 0
OFFSET_THETA = 0


def main():
	""" This function moves the turtle in a circle """
	global ROTATION_COUNT, PI
	rospy.Subscriber("/turtle1/pose", Pose, callback)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	cmd = Twist()
	time_period = 3
	radius = 2
	num_turns = 1
	cmd.linear.x = (TWO_PI * radius) / time_period
	cmd.linear.y = 0
	cmd.linear.z = 0
	cmd.angular.x = 0
	cmd.angular.y = 0
	cmd.angular.z = TWO_PI / time_period
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		if ROTATION_COUNT == num_turns:
			cmd.linear.x = 0
			cmd.angular.z = PI / 2
			pub.publish(cmd)
			break
		pub.publish(cmd)
		rate.sleep()


def callback(pos):
	""" This function is called everytime position data is published on /turtle1/pose topic """
	global RADIANS_COVERED, ROTATION_COUNT, PREV_THETA, OFFSET_THETA, ONCE

	if ONCE:
		OFFSET_THETA = -pos.theta
		ONCE = 0

	# Now initial orientation position is taken as 0 rad
	RADIANS_COVERED = pos.theta + OFFSET_THETA

	# condition for completion of revolution when angular velocity is +ve
	if PREV_THETA < 0 and RADIANS_COVERED >= 0:
		ROTATION_COUNT += 1
	if RADIANS_COVERED < 0:  # if theta < 0 that means that it is in b/w pi and 2pi
		RADIANS_COVERED += TWO_PI * (ROTATION_COUNT + 1)

	# if theta > 0 that means its b/w 0 and pi so when ROTATION_COUNT is 0 nothing is added to theta
	else:
		RADIANS_COVERED += TWO_PI * ROTATION_COUNT

	PREV_THETA = pos.theta + OFFSET_THETA
	rospy.loginfo("round and round we go, {} rad".format(RADIANS_COVERED))


if __name__ == '__main__':
	rospy.init_node('node_turtle_revolve')
	main()
