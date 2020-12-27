#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode


class Camera1:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
		self.qr_threshold = 10
		self.packages_info = {}
	
	def get_qr_data(self, arg_image):
		qr_codes = decode(arg_image)
		if len(qr_codes) == 12:
			for qr in qr_codes:
				package_name = self.get_package_name(qr.rect)
				self.packages_info[package_name] = qr.data
			return 0
		else:
			return -1

	
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		roi = cv_image[290:920, 100:620]
		mask = cv2.inRange(roi,(0,0,0), (self.qr_threshold, self.qr_threshold, self.qr_threshold))
		thresholded = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
		roi = 255-thresholded

		self.get_qr_data(roi)

	def get_package_name(self, coordinates):
		package_index = ''
		col, row = coordinates[0], coordinates[1]

		if col < 174:
			package_index = '0'
		elif col < 347:
			package_index = '1'
		else:
			package_index = '2'

		if row < 158:
			package_index += '0'
		elif row < 315:
			package_index += '1'
		elif row < 473:
			package_index += '2'
		else:
			package_index+= '3'

		package_index = package_index[::-1]

		return 'packagen'+package_index

def main():

	rospy.init_node('node_get_package_info', anonymous=True)
	try:
		ic = Camera1()
		while not ic.packages_info:
			rospy.sleep(0.1)
		rospy.set_param('/packages_color_info', ic.packages_info)
		ic.image_sub.unregister()
	except Exception as e:
		print(e)
	print(ic.packages_info)

if __name__ == '__main__':
	main()
