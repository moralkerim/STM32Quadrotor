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

x_len = 200         # Number of points to display
y_range = [-30, 30]  # Range of possible Y values to display

time = list(range(0, 200))
roll =  [0] * x_len
pitch = []
i = 0;
style.use('fivethirtyeight')

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.set_ylim(y_range)
line, = ax.plot(time, roll)


    
print("server {} portunu dinliyor.".format(port))

def animate(i,roll):

    (data, addr) = mySocket.recvfrom(1024)


    data_org = struct.unpack('<fffHHHH', data)
    print(data_org)
    #print(data)

    roll.append(data_org[0])   
    roll = roll[-x_len:]
    line.set_ydata(roll)
    return line,
    
ani = animation.FuncAnimation(fig, animate, fargs=(roll,), interval=50, blit=True)
plt.show()




