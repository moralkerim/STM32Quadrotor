import socket
import sys
import time
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

port = 9000  # haberleşilen port adresi
ip = ''
mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mySocket.bind((ip, port))
time = []
roll = []
pitch = []
i = 0;
style.use('fivethirtyeight')

fig = plt.figure()
ax = fig.add_subplot(1,1,1)



    
print("server {} portunu dinliyor.".format(port))

while 1:

    (data, addr) = mySocket.recvfrom(1024)

    try:
        data_org = struct.unpack('<fffHHHH', data)
        print(data_org)
        #print(data)

    except:
        print("Error.")
        print(data)
        




