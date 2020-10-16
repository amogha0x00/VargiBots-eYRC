
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


## MY_SHEET_ID = "AKfycbyfj6ZQSCxddxTHoFAy1L-CADEMl_X6psDD3tsIZ_XHUEVdIoA"

def update_spreadsheet(*args):
	titles = ['id', 'team_id', 'unique_id']
	parameters = {}
	for row in args:
		url = "https://script.google.com/macros/s/" + row[0] + "/exec"
		for index, value in enumerate(row[1:4]):
			parameters[titles[index]] = value
		parameters.update(literal_eval(row[4]))
		response = requests.get(url, params=parameters)
		print('Made A GET request to :\n url: {}\n params: {}'.format(url, parameters))
		print(response.content)
	#print(not response.status_code == 200)
	return not response.status_code == 200

#update_spreadsheet(("AKfycbyfj6ZQSCxddxTHoFAy1L-CADEMl_X6psDD3tsIZ_XHUEVdIoA", 'task1', '1823', 'doesnt', "{'turtle_x':145, 'turtle_y':906,'turtle_theta':5688}"),
#					("AKfycbyfj6ZQSCxddxTHoFAy1L-CADEMl_X6psDD3tsIZ_XHUEVdIoA", 'task1', '1823', 'doesnt', "{'turtle_x':85, 'turtle_y':336,'turtle_theta':576}"))
