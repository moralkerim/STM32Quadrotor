import socket
import sys
import time
import struct


port = 9000  # haberleşilen port adresi
ip = ''
mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mySocket.bind((ip, port))


print("server {} portunu dinliyor.".format(port))

(data, addr) = mySocket.recvfrom(1024)
data_org = struct.unpack('<HHBBBBddfffffH', data)

print(data)
print(data_org)
