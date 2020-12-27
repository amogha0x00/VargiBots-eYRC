#!/usr/bin/env python
import rospy

def main():
	rospy.init_node('node_task4', anonymous=True)
	try:
		while not rospy.has_param('/packages_color_info'):
			rospy.sleep(0.1)
	except KeyboardInterrupt:
		print('ok quiting')
	info = rospy.get_param('/packages_color_info')
	print(info)

main()