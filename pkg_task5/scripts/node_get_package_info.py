#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import cv2


class Camera1:

	def __init__(self):
		self.bridge = CvBridge()
		self.qr_threshold = 10
		self.packages_info = {}
		rospy.sleep(0.1)
		self._start_time = rospy.get_time()
		self._current_time = rospy.get_time()
		self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)

	def get_qr_data(self, arg_image):
		"""
			Decodes the qr code data in preprossed image and updates `packages_info` dict
			with corresponding package names and qr code decoded data
		"""
		qr_codes = decode(arg_image)

		if len(qr_codes) == 12:
			for qr in qr_codes:
				package_name = self.get_package_name(qr.rect)
				self.packages_info[package_name] = qr.data
			return 0
		return -1


	def callback(self, data):
		"""
			This is callback function for camera1 it gets image data through argument`data`
			it thersholds the image with lower bound `(0, 0, 0)` and upper bound set by
			`self.qr_threshold` the it inverts image and feeds it to qr_decode func to
			decode the qr codes
		"""
		self._current_time = rospy.get_time()
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		roi = cv_image[290:920, 100:620] # crop to only package area
		lower_bound = (0, 0, 0)
		if (self._current_time - self._start_time) > 15: # if 15 sec is passed after launching the node then
			self.qr_threshold += 1					   # change the threshold value till all packages are detected
			self.qr_threshold %= 256
			rospy.logwarn("Default Config Not Working Trying New Configs: {}".format(self.qr_threshold))
		upper_bound = (self.qr_threshold, self.qr_threshold, self.qr_threshold)
		thresholded = cv2.inRange(roi, lower_bound, upper_bound)
		inv = 255-thresholded
		if not self.get_qr_data(inv):
			self.image_sub.unregister()

	def get_package_name(self, coordinates):
		"""
			Returs the name of the packages based on the coordinates
			of the top left corner of the decoded qr code given
			through `coordinates`
		"""
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
			package_index += '3'

		package_index = package_index[::-1]

		return 'packagen'+package_index

def main():

	rospy.init_node('node_get_package_info', anonymous=True)
	try:
		cam = Camera1()
		while not cam.packages_info:
			rospy.sleep(0.1)
		rospy.set_param('/packages_color_info', cam.packages_info)
	except Exception as e:
		print(e)
	rospy.logwarn(cam.packages_info)

if __name__ == '__main__':
	main()
