import socket
import sys
import time
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from datetime import datetime
from enum import Enum
import select
telem_value = 1

class Telem(Enum):
      roll=1
      pitch=2
      yaw=3
      roll_des=4
      pitch_des=5
      yaw_des=6
      roll_rate=7
      pitch_rate=8
      yaw_rate=9
      roll_des_rate=10
      pitch_des_rate=11
      yaw_des_rate=12
      w1=13
      w2=14
      w3=15
      w4=16
      roll_acc=17
      pitch_acc=18
      roll_gyro=19
      pitch_gyro=20
      roll_comp=21
      pitch_comp=22
      roll_ekf=23
      pitch_ekf=24
      P_roll=25
      I_roll=26
      D_roll=27
      pid_roll=28
      P_pitch=29
      I_pitch=30
      D_pitch=31
      pid_pitch=32
      sonar_alt=33
      alt_thr = 34
      xb = 35
      yb = 36
      zb = 37
      vx = 38
      vy = 39
      vz = 40
      time_millis=41
      detected=42
      xcam = 43
      ycam = 44
      zcam = 45
      yawcam = 46
      accX = 47
      accY = 48
      accZ = 49
      magX = 50
      magY = 51
      magZ = 52
      lat = 53
      lon = 54
      alt_gps = 55
      xned = 56
      yned = 57
      zned = 58
      xbody_gps = 59
      ybody_gps = 60
      zbody_gps = 61
      vxbody_gps = 62
      vybody_gps = 63
      vzbody_gps = 64
      ch1 = 65
      ch2 = 66
      ch3 = 67
      ch4 = 68
      ch5 = 69
      ch6 = 70
      ch7 = 71
      ch9 = 72
      ch8 = 73
      pwm2_1 = 74
      pwm2_2 = 75
      pwm2_3 = 76
      pwm2_4 = 77

exit = True      
for telem in Telem:
        if (sys.argv[1] == telem.name):
            telem_value = telem.value
            exit = False
            break
 
if(exit):
    sys.exit("Girilen telemetri degeri mevcut degil.")

now = datetime.now()
dt_string = now.strftime("%d.%m.%Y.%H.%M.%S")
print("date and time =", dt_string)	


if(int(sys.argv[4]) == 1):
    port = 9000  # haberleşilen port adresi
    
else:
    port = 9001
ip = ''
mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#mySocket.setblocking(False)
mySocket.bind((ip, port))


x_len = 200      # Number of points to display
y_range = [int(sys.argv[2]), int(sys.argv[3])]  # Range of possible Y values to display

time = list(range(0, x_len))
data_enum =  [0] * x_len
i = 0;
style.use('fivethirtyeight')

fig = plt.figure()
fig.set_size_inches(10, 5, forward=True)
ax = fig.add_subplot(1,1,1)
#ay = fig.add_subplot(2,1,2)
ax.set_ylim(y_range)
#ay.set_ylim(y_range)
linex, = ax.plot(time, data_enum)
#liney, = ay.plot(time, pitch,'tab:red')
ax.set_title("Data")
ax.set(ylabel = "Value")
#ay.set_title("Pitch")

    
print("server {} portunu dinliyor.".format(port))

def empty_socket(sock):
    """remove the data present on the socket"""
    input = [sock]
    while 1:
        inputready, o, e = select.select(input,[],[], 0.0)
        if len(inputready)==0: break
        for s in inputready: s.recv(1024)


def animate(i,data_enum):
        (data, addr) = mySocket.recvfrom(1024)

        empty_socket(mySocket)
        data_org = struct.unpack('<ffffffffffffHHHHffffffffffffffffffffffffLBhhhhfffhhhffffffffffffHHHHHHHHHHHHH', data)

        data_enum.append(data_org[telem_value-1])  
        #pitch.append(data_org[1])
        
        data_enum = data_enum[-x_len:]
        #pitch = pitch[-x_len:]
        
        linex.set_ydata(data_enum)
        #liney.set_ydata(pitch)
        return linex,
    
ani = animation.FuncAnimation(fig, animate, fargs=(data_enum,), interval=0.05, blit=True)
plt.show()