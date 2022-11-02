import paho.mqtt.client as mqtt #import the client1
import time
import struct
import sys
from datetime import datetime

############
def on_message_ch(client, userdata, message):

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
def on_message_att(client, userdata, message):

   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:12]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<fff', buf)
    #with open(dt_string + '.txt', "a") as file:
        #file.write("\n")
        #file.write(str(data_org))
    print(data_org)
   except:
       print("cant decode.")
       print(message.payload)
       print(buf)
def on_message_acc(client, userdata, message):

   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:12]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<fff', buf)
    #with open(dt_string + '.txt', "a") as file:
        #file.write("\n")
        #file.write(str(data_org))
    print(data_org)
   except:
       print("cant decode.")
       print(message.payload)
       print(buf)
def on_message_cam(client, userdata, message):

   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:11]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<BHHHHH', buf)
    #with open(dt_string + '.txt', "a") as file:
        #file.write("\n")
        #file.write(str(data_org))
    print(data_org)
   except:
       print("cant decode.")
       print(message.payload)
       print(buf)
def on_message_mag(client, userdata, message):

   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:6]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<HHH', buf)
    #with open(dt_string + '.txt', "a") as file:
        #file.write("\n")
        #file.write(str(data_org))
    print(data_org)
   except:
       print("cant decode.")
       print(message.payload)
       print(buf)
def on_message_ekf(client, userdata, message):

   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:32]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<ffffffff', buf)
    #with open(dt_string + '.txt', "a") as file:
        #file.write("\n")
        #file.write(str(data_org))
    print(data_org)
   except:
       print("cant decode.")
       print(message.payload)
       print(buf)
def on_message_pid_roll(client, userdata, message):

   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:32]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<ffff', buf)
    #with open(dt_string + '.txt', "a") as file:
        #file.write("\n")
        #file.write(str(data_org))
    print(data_org)
   except:
       print("cant decode.")
       print(message.payload)
       print(buf)
def on_message_pid_pitch(client, userdata, message):

   #print(str(message.payload.decode("utf-8")))
    #data = (str(message.payload.decode("utf-8")))
   buf = message.payload[0:32]
   #print(sys.getsizeof(buf))
   try:
    data_org = struct.unpack('<ffff', buf)
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

broker_address="192.168.1.37"
port = 1883
#broker_address="iot.eclipse.org"
print("creating new instance")
client = mqtt.Client("P1") #create new instance

###########################################
#client.message_callback_add('ch', on_message_ch)

client.message_callback_add('telem/att', on_message_att)
#client.message_callback_add('telem/acc', on_message_acc)
# client.message_callback_add('telem/cam', on_message_cam)
# client.message_callback_add('telem/mag', on_message_mag)
# client.message_callback_add('telem/ekf', on_message_ekf)
# client.message_callback_add('telem/pid_roll', on_message_pid_roll)
# client.message_callback_add('telem/pid_pitch', on_message_pid_pitch)
############################################

print("connecting to broker")

client.connect(broker_address,port) #connect to broker
client.loop_start() #start the loop
print("Subscribing to topics")
client.subscribe("telem/#")
client.subscribe("ch")
#print("Publishing message to topic","house/bulbs/bulb1")
#client.publish("house/bulbs/bulb1","OFF")
time.sleep(4) # wait
#client.loop_stop() #stop the loop

client.loop_forever()