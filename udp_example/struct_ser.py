import socket
import sys
import time
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from datetime import datetime
from threading import Timer


    

now = datetime.now()
dt_string = now.strftime("%d.%m.%Y.%H.%M.%S")
print("date and time =", dt_string)	

port = 9000  # haberleşilen port adresi
ip = ''
mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mySocket.bind((ip, port))

x_len = 200         # Number of points to display
y_range = [-10, 10]  # Range of possible Y values to display

time = list(range(0, x_len))
roll =  [0] * x_len
pitch = [0] * x_len
i = 0;
style.use('fivethirtyeight')

fig = plt.figure()
fig.set_size_inches(10, 5, forward=True)
ax = fig.add_subplot(2,1,1)
ay = fig.add_subplot(2,1,2)
ax.set_ylim(y_range)
ay.set_ylim(y_range)
linex, = ax.plot(time, roll)
liney, = ay.plot(time, pitch,'tab:red')
ax.set_title("Roll")
ax.set(ylabel = "Degrees")
ay.set_title("Pitch")

def Log():

    (data, addr) = mySocket.recvfrom(1024)


    data_org = struct.unpack('<ffffffffffffHHHHfffffffffffff', data)
    #file.write("\n")
    #file.write(str(data_org))
#    print(data_org)
    #print(data)

    
print("server {} portunu dinliyor.".format(port))
file = open(dt_string + '.txt', "a");


t = Timer(0.01, Log)
t.start()  # After 0.05 seconds, "Hello" will be printed

    
#ani = animation.FuncAnimation(fig, animate, fargs=(roll, pitch,), interval=50, blit=True)
#plt.show()




