import paho.mqtt.client as mqtt #import the client1
import time
import struct
import sys
from datetime import datetime

############

def on_message(client, userdata, message):
   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:8]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<HHHH', buf)
    #with open(dt_string + '.txt', "a") as file:
        #file.write("\n")
        #file.write(str(data_org))
    print(data_org)
   except:
       print("cant decode.")
       print(message.payload)
       print(buf)
   #print(message.payload)
   #print(buf)
########################################
now = datetime.now()
dt_string = now.strftime("%d.%m.%Y.%H.%M.%S")

broker_address="192.168.1.41"
#broker_address="iot.eclipse.org"
print("creating new instance")
client = mqtt.Client("P1") #create new instance
client.on_message=on_message #attach function to callback
print("connecting to broker")
client.connect(broker_address) #connect to broker
client.loop_start() #start the loop
print("Subscribing to topic","ch")
client.subscribe("ch")
#print("Publishing message to topic","house/bulbs/bulb1")
#client.publish("house/bulbs/bulb1","OFF")
time.sleep(4) # wait
#client.loop_stop() #stop the loop

while True:
    client.loop
    time.sleep(0.1)