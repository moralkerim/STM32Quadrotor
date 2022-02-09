import socket
import sys
import time
import struct
import serial

port = 9000  # haberleşilen port adresi
ip = ''
mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mySocket.bind((ip, port))
#ser = serial.Serial('COM3',115200)

sent = False

print("server {} portunu dinliyor.".format(port))

(data, addr) = mySocket.recvfrom(1024)

try:
    data_org = struct.unpack('<fffHHHH', data)
    print(data_org)
    print(data)


except:
    print("Error.")
    print(data)




#if(not sent):
   # ser.write(data)


