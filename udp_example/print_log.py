import socket
import sys
import time
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from datetime import datetime

now = datetime.now()
dt_string = now.strftime("%d.%m.%Y.%H.%M.%S")
print("date and time =", dt_string)	

port = 9000  # haberleşilen port adresi
ip = ''
mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mySocket.bind((ip, port))




while True:
    (data, addr) = mySocket.recvfrom(1024)
    data_org = struct.unpack('<ffffffffffffHHHHffffffffffffffffffffffffLBhhhhfff', data)
    with open(dt_string + '.txt', "a") as file:
        file.write("\n")
        file.write(str(data_org))
    print(data_org)
    #print(data)
