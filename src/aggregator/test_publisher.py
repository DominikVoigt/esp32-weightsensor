import paho.mqtt.client as mqtt
import time
import random
import struct

ipAddressMQTT = "192.168.178.24"
portMQTT = 1900


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(ipAddressMQTT, 1900, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
while True:
    time.sleep(0.1)
    choice = random.choice([0,1,2,3,4,5,6,7,8,9,10,11])
    client.publish("cocktail/weight/sensor" + str(choice), struct.pack('<i', int(random.uniform(0, 150))))
    