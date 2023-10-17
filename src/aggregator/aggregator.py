import paho.mqtt.client as mqtt
import jsonpickle
from weight_dto import Weight_DTO
from flask import Flask, make_response
from markupsafe import escape
import struct

app = Flask(__name__)
ipAddressMQTT = "192.168.0.118"
portMQTT = 1900

sensorData = {}

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("cocktail/weight/+")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    sensor = msg.topic[msg.topic.rfind('/')+1:]
    weight = struct.unpack('<i', msg.payload)
    print(f"Received payload weight: {weight[0]}")
    if(sensor in sensorData.keys()):
        sensorData[sensor].weight = weight[0]
    else:
        sensorData[sensor] = Weight_DTO(sensor, weight[0])

# This code has to be put here to make it run with the flask run command
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(ipAddressMQTT, portMQTT, 60)
app.debug=True
client.loop_start()

@app.route("/", methods=['GET'])
def home():
    endpoints = {
        "GET" : {
            "weights": "Data of all sensors that data was aggregated on",
            "weights/<sensor-id>": "Data of a particular sensor specified by the <sensor-id> can be seen under the name property of the weight endpoint",
            "weights/<sensor-id>/name": "Name of the sensor, corresponds with <sensor-id>",
            "weights/<sensor-id>/weight": "Latest published weight of the sensor",
            "weights/<sensor-id>/full": "Weight of the completely filled container on the scale. Null if weight not set previously via a POST to this endpoint",
            "weights/<sensor-id>/empty": "Weight of the completely empty container on the scale. Null if weight not set previously via a POST to this endpoint",
            "weights/<sensor-id>/level": "Current fill level based on weight of the container on the scale, requires full and empty to be set!",
        },
        "POST" : {
            "weights/<sensor-id>/full": "Sets the full value to the currently published weight. Used to calibrate the weight of a completely filled container on the scale.",
            "weights/<sensor-id>/empty": "Sets the empty value to the currently published weight. Used to calibrate the weight of a completely empty container on the scale.",
        }
        }
    return endpoints

@app.route("/weights", methods=['GET'])
def getAllWeightData():
    sensorDataJson = jsonpickle.encode(sensorData, unpicklable=False)
    response = make_response(sensorDataJson)
    response.content_type = 'application/json'
    return response

@app.route("/weights/<string:sensor>", methods=['GET'])
def getSensorData(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():
        response = make_response('The requested sensor is not registered.', 404)
        return response
    sensorDataJson = jsonpickle.encode(sensorData.get(sensor), unpicklable=False)
    response = make_response(sensorDataJson)
    response.content_type = 'application/json'
    return response

@app.route("/weights/<string:sensor>/name", methods=['GET'])
def getSensorName(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():
        response = make_response('The requested sensor is not registered.', 404)
        return response
    return sensorData.get(sensor).name

@app.route("/weights/<string:sensor>/weight", methods=['GET'])
def getWeightData(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():
        response = make_response('The requested sensor is not registered.', 404)
        return response
    return str(sensorData.get(sensor).weight)

@app.route("/weights/<string:sensor>/full", methods=['GET'])
def getFullData(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():
        response = make_response('The requested sensor is not registered.', 404)
        return response
    fullWeight = sensorData.get(sensor).full
    return str(fullWeight) if fullWeight is not None else 'null' 

@app.route("/weights/<string:sensor>/empty", methods=['GET'])
def getEmptyData(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():
        response = make_response('The requested sensor is not registered.', 404)
        return response
    emptyWeight = sensorData.get(sensor).empty
    return str(emptyWeight) if emptyWeight is not None else 'null'
    
@app.route("/weights/<string:sensor>/level", methods=['GET'])
def getContainerFillLevel(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():        
        response = make_response('The requested sensor is not registered.', 404)
        return response
    data = sensorData[sensor]
    if data.empty == None: 
        response = make_response('The empty container has to be weight first using the empty endpoint.', 424)
        return response
    if data.full == None: 
        response = make_response('The full container has to be weight first using the full endpoint.', 424)
        return response
    return str((data.weight - data.empty) / (data.full - data.empty))
    

# Add calibrate between full/empty or not?

@app.route("/weights/<string:sensor>/full", methods=['POST'])
def setFullContainerWeight(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():
        raise KeyError("The requested sensor is not registered.")
    sensorData[sensor].full = sensorData[sensor].weight
    return str(sensorData[sensor].full)

@app.route("/weights/<string:sensor>/empty", methods=['POST'])
def setEmptyContainerWeight(sensor):
    sensor = escape(sensor)
    if sensor not in sensorData.keys():
        raise KeyError("The requested sensor is not registered.")
    sensorData[sensor].empty = sensorData[sensor].weight
    return str(sensorData[sensor].empty)