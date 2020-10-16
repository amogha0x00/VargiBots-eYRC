
import paho.mqtt.client as mqtt
import time
from ast import literal_eval
import requests


def on_connect(client, userdata, flags, rc):
   print("Connected With Result Code "+str(rc))


def mqtt_subscribe_thread_start(on_message, broker_url, broker_port, sub_topic, qos):
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message
	client.connect(broker_url, broker_port)
	(rc, mid) = client.subscribe(sub_topic, qos=qos)
	client.loop_start()
	return rc


def mqtt_publish(broker_url, broker_port, pub_topic, payload, qos):
	client = mqtt.Client()
	client.connect(broker_url, broker_port)
	client.loop_start()
	pub_info = client.publish(pub_topic, payload, qos=qos, retain=False)
	pub_info.wait_for_publish()
	#time.sleep(2)
	client.loop_stop()
	return pub_info[0]


def update_spreadsheet(spread_sheet_id, sheet_name, team_id, unique_id, data_points_dict_str):
	parameters = {'id':sheet_name, 'team_id':team_id, 'unique_id':unique_id}
	url = "https://script.google.com/macros/s/" + spread_sheet_id + "/exec"
	
	data_points_dict = literal_eval(data_points_dict_str)
	parameters.update(data_points_dict)
	
	response = requests.get(url, params=parameters)
	print('Made A GET request to :\n url: {}\n params: {}'.format(url, parameters))
	print(response.content)
	return not response.status_code == 200

