
import paho.mqtt.client as mqtt


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
	pub_info = client.publish(pub_topic, payload, qos=qos, retain=False)
	pub_info.wait_for_publish()
	return pub_info[0]