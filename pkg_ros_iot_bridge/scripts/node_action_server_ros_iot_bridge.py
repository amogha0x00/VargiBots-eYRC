#!/usr/bin/env python

"""
	Team ID: VB#1823
	Theme: Vergi Bots

	This node can accept number of Goals from any number of Action Clients.
	On receiving appropriate Goal the Action Server,
    * can push as many data points on a Google Sheet at once.
    * can publish on any MQTT Topic.
    * can subscribe to any MQTT Topic and publish the message from MQTT subscription
	  to a ROS Topic sub_cb_ros_topic specified in config_pyiot.yaml
	  using msgMqttSub.msg message file.
"""

# ROS Node - Action Server - ROS IoT Bridge

import threading
from ast import literal_eval
import rospy
import actionlib

from pkg_ros_iot_bridge.msg import msgRosIotAction  # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal  # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult  # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback  # Message Class that is used for Feedback Messages

from pkg_ros_iot_bridge.msg import msgMqttSub  # Message Class for MQTT Subscription Messages

from pyiot import iot  # Custom Python Module to perfrom MQTT Tasks

class ActionServerRosIoTBridge:

	# Constructor
	def __init__(self):
		# Initialize the Action Server
		self._as = actionlib.ActionServer('/action_ros_iot',
										  msgRosIotAction,
										  self.on_goal,
										  self.on_cancel,
										  auto_start=False)

		'''
			* self.on_goal - It is the fuction pointer which points to a function which will be called
							 when the Action Server receives a Goal.

			* self.on_cancel - It is the fuction pointer which points to a function which will be called
							 when the Action Server receives a Cancel Request.
		'''

		# Read and Store IoT Configuration data from Parameter Server
		param_config_pyiot = rospy.get_param('config_pyiot')
		self._config_mqtt_server_url = param_config_pyiot['mqtt']['server_url']
		self._config_mqtt_server_port = param_config_pyiot['mqtt']['server_port']
		self._config_mqtt_sub_topic = param_config_pyiot['mqtt']['topic_sub']
		self._config_mqtt_pub_topic = param_config_pyiot['mqtt']['topic_pub']
		self._config_mqtt_qos = param_config_pyiot['mqtt']['qos']
		self._config_mqtt_sub_cb_ros_topic = param_config_pyiot['mqtt']['sub_cb_ros_topic']


		print('\n\nPublish msg as "start" on  : {}'.format(self._config_mqtt_sub_topic))
		print('Subscribe to receive result on : {}\n'.format(self._config_mqtt_pub_topic))

		# Initialize ROS Topic Publication_config_mqtt_sub_cb_ros_topic
		# Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
		# ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
		self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

		# Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_pyiot.yaml'.
		# self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
		ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
											  self._config_mqtt_server_url,
											  self._config_mqtt_server_port,
											  self._config_mqtt_sub_topic,
											  self._config_mqtt_qos)
		if ret == 0:
			rospy.loginfo("MQTT Subscribe Thread Started")
		else:
			rospy.logerr("Failed to start MQTT Subscribe Thread")

		# Start the Action Server
		self._as.start()

		rospy.loginfo("Started ROS-IoT Bridge Action Server.")

	def mqtt_sub_callback(self, client, userdata, message):
		"""
			This is a callback function for MQTT Subscriptions
		"""
		payload = str(message.payload.decode("utf-8"))

		print("[MQTT SUB CB] Message: ", payload)
		print("[MQTT SUB CB] Topic: ", message.topic)

		msg_mqtt_sub = msgMqttSub()
		msg_mqtt_sub.timestamp = rospy.Time.now()
		msg_mqtt_sub.topic = message.topic
		msg_mqtt_sub.message = payload

		self._handle_ros_pub.publish(msg_mqtt_sub)

	def on_goal(self, goal_handle):
		"""
			This function will be called when Action Server receives a Goal
		"""
		goal = goal_handle.get_goal()

		rospy.loginfo("Received new goal from Client")
		rospy.loginfo(goal)

		# Validate incoming goal parameters
		if goal.protocol == "mqtt":

			if (goal.mode == "pub") or (goal.mode == "sub"):
				goal_handle.set_accepted()

				# Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
				# 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
				thread = threading.Thread(name="worker",
										  target=self.process_goal,
										  args=(goal_handle,))
				thread.start()

			else:
				goal_handle.set_rejected()
				return

		elif goal.protocol == 'http':
			goal_handle.set_accepted()

			# Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
			# 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
			thread = threading.Thread(name="worker",
									  target=self.process_goal,
									  args=(goal_handle,))
			thread.start()
		else:
			goal_handle.set_rejected()
			return

	def process_goal(self, goal_handle):
		"""
			This function is called is a separate thread to process Goal.
		"""
		flag_success = False
		result = msgRosIotResult()

		goal_id = goal_handle.get_goal_id()
		rospy.loginfo("Processing goal : " + str(goal_id.id))

		goal = goal_handle.get_goal()

		# Goal Processing
		if goal.protocol == "mqtt":
			rospy.logwarn("MQTT")

			if goal.mode == "pub":
				rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

				rospy.logwarn(goal.topic + " > " + goal.message)

				ret = iot.mqtt_publish(self._config_mqtt_server_url,
									   self._config_mqtt_server_port,
									   goal.topic,
									   goal.message,
									   self._config_mqtt_qos)

				if ret == 0:
					rospy.loginfo("MQTT Publish Successful.")
					result.flag_success = True
				else:
					rospy.logerr("MQTT Failed to Publish")
					result.flag_success = False

			elif goal.mode == "sub":
				rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
				rospy.logwarn(goal.topic)

				ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
													  self._config_mqtt_server_url,
													  self._config_mqtt_server_port,
													  goal.topic,
													  self._config_mqtt_qos)
				if ret == 0:
					rospy.loginfo("MQTT Subscribe Thread Started")
					result.flag_success = True
				else:
					rospy.logerr("Failed to start MQTT Subscribe Thread")
					result.flag_success = False

		elif goal.protocol == "http":
			rospy.logwarn("HTTP")
			
			rospy.logwarn("HTTP GET Goal ID: " + str(goal_id.id))

			rospy.logwarn(goal.topic + " > " + goal.message)

			# update_spreadsheet(spreadsheet_id, id, 'team_id', 'unique_id', '{'turtle_x': x, 'turtle_y': y, 'turtle_theta': theta,...,..}')
			ret = iot.update_spreadsheet(goal.topic, # using topic to send spreadsheet_id 
										goal.mode,
										literal_eval(goal.message) # data points with column names in dict format str
										)

			if ret == 0:
				rospy.loginfo("Updating Spreadsheet Successful.")
				result.flag_success = True
			else:
				rospy.logerr("Updating Spreadsheet Failed")
				result.flag_success = False

		rospy.loginfo("Send goal result to client")
		if result.flag_success == True:
			rospy.loginfo("Succeeded")
			goal_handle.set_succeeded(result)
		else:
			rospy.loginfo("Goal Failed. Aborting.")
			goal_handle.set_aborted(result)

		rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

	def on_cancel(self, goal_handle):
		"""
			This function will be called when Goal Cancel request is send to the Action Server
		"""
		rospy.loginfo("Received cancel request.")
		goal_id = goal_handle.get_goal_id()


# Main
def main():
	rospy.init_node('node_action_server_ros_iot_bridge')

	action_server = ActionServerRosIoTBridge()

	rospy.spin()


if __name__ == '__main__':
	main()
