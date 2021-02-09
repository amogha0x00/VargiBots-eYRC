"""
	Team ID: VB#1823
	Theme: Vergi Bots

	This module contains all the necessary functions for IoT Tasks (MQTT & HTTP).
	It is imported by `node_action_server_ros_iot_bridge.py` for IoT tasks. 
"""

import paho.mqtt.client as mqtt
import requests


def on_connect(client, userdata, flags, rc):
	print("Connected With Result Code "+str(rc))


def mqtt_subscribe_thread_start(on_message, broker_url, broker_port, sub_topic, qos):
	"""
		This function subscribes to any mqtt topic given on "sub_topic" and makes "on_message" as on_message cb function
	"""
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message
	client.connect(broker_url, broker_port)
	(rc, mid) = client.subscribe(sub_topic, qos=qos)
	client.loop_start()
	return rc


def mqtt_publish(broker_url, broker_port, pub_topic, payload, qos):
	"""
		This function publish's on any mqtt topic given on "pub_topic" with message given as "payload"
	"""
	client = mqtt.Client()
	client.connect(broker_url, broker_port)
	client.loop_start()
	pub_info = client.publish(pub_topic, payload, qos=qos, retain=False)
	pub_info.wait_for_publish()
	client.loop_stop()
	return pub_info[0]


def update_spreadsheet(spread_sheet_id, sheet_name, data_points_dict):
	"""
		This function makes a GET request to any webapp of given "spread_sheet_id" with any number of datapoints 
		given through "data_points_dict"
		ex: data_points_dict = { 'turtle_x': 10, 'turtle_y': 10, 'turtle_theta': 2} pushes 10 to turtle_x and turtle_y column
		and 2 to turtle_theta column in this format there could be any number of data points
	"""
	parameters = {'id':sheet_name, 'Team Id':'VB#1823', 'Unique Id':'EsNEciqV'}
	url = "https://script.google.com/macros/s/" + spread_sheet_id + "/exec"

	parameters.update(data_points_dict) # appends datapoints to parameters

	response = requests.get(url, params=parameters)
	print('Made A GET request to :\n url: {}\n params: {}'.format(url, parameters))
	print(response.content)
	return not response.status_code == 200
