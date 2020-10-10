#!/usr/bin/env python

'''

Team id : 1823
Theme :  Vargi Bots (VB)
Author : Amoghavarsha S G

'''

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

PI = 3.14159265
ONCE = 1
radians_covered = 0
rotation_count = 0
prev_theta = 0
offset_theta = 0


def main():
	global rotation_count, PI
	rospy.Subscriber("/turtle1/pose", Pose, callback)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	cmd = Twist()
	time_period = 2
	radius = 2
	num_turns = 1
	cmd.linear.x = (2*PI*radius)/time_period
	cmd.linear.y = 0
	cmd.linear.z = 0
	cmd.angular.x = 0
	cmd.angular.y = 0
	cmd.angular.z = 2*PI/time_period
	rate = rospy.Rate(100)
	while (not rospy.is_shutdown()):
		if (rotation_count == num_turns):
			cmd.linear.x = 0
			cmd.angular.z = PI/2
			pub.publish(cmd)
			break
		pub.publish(cmd)
		rate.sleep()


def callback(pos):
	global radians_covered, rotation_count, prev_theta, offset_theta ,ONCE
	
	if ONCE:
		offset_theta = -pos.theta
		ONCE = 0

	radians_covered = pos.theta + offset_theta
	
	if  prev_theta < 0 and radians_covered >= 0:
		rotation_count += 1
	if radians_covered < 0: # if theta < 0 that means that it is in b/w pi and 2pi  
		radians_covered += 2*PI * (rotation_count+1) 
	else: # if theta > 0 that means that it is in b/w 0 and pi so when rotation_count is 0 nothing is added to theta
		radians_covered += 2*PI * rotation_count
	
	prev_theta = pos.theta + offset_theta
	rospy.loginfo("round and round we go, {} rad".format(radians_covered))

if __name__ == '__main__':
	rospy.init_node('turtle_revolve')
	main()
