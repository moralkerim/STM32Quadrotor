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
      yaw_acc = 19
      roll_gyro=20
      pitch_gyro=21
      yaw_gyro=22
      roll_comp=23
      pitch_comp=24
      roll_ekf=25
      pitch_ekf=26
      yaw_rate_ekf=27
      P_roll=28
      I_roll=29
      D_roll=30
      pid_roll=31
      P_pitch=32
      I_pitch=33
      D_pitch=34
      pid_pitch=35
      sonar_alt=36
      alt_thr = 37
      xb = 38
      yb = 39
      zb = 40
      vx = 41
      vy = 42
      vz = 43
      time_millis=44
      detected=45
      xcam = 46
      ycam = 47
      zcam = 48
      yawcam = 49
      accX = 50
      accY = 51
      accZ = 52
      magX = 53
      magY = 54
      magZ = 55
      lat = 56
      lon = 57
      alt_gps = 58
      xned = 59
      yned = 60
      zned = 61
      xbody_gps = 62
      ybody_gps = 63
      zbody_gps = 64
      vxbody_gps = 65
      vybody_gps = 66
      vzbody_gps = 67
      mag = 68
      pwm2_1 = 69
      pwm2_2 = 70
      pwm2_3 = 71
      pwm2_4 = 72
      P_yaw=73
      I_yaw=74
      D_yaw=75
      pid_yaw=76
      
      S_roll_11=77
      S_roll_12=78
      S_roll_13=79
      S_roll_21=80
      S_roll_22=81
      S_roll_23=82
      S_roll_31=83
      S_roll_32=84
      S_roll_33=85
      
      S_pitch_11=86
      S_pitch_12=87
      S_pitch_13=88
      S_pitch_21=89
      S_pitch_22=90
      S_pitch_23=91
      S_pitch_31=92
      S_pitch_32=93
      S_pitch_33=94
      
      S_yaw_11=95
      S_yaw_12=96
      S_yaw_13=97
      S_yaw_21=98
      S_yaw_22=99
      S_yaw_23=100
      S_yaw_31=101
      S_yaw_32=102
      S_yaw_33=103


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
        
        try:
            data_org = struct.unpack('<ffffffffffffHHHHffffffffffffffffffffffffffLBhhhhfffhhhffffffffffffHHHHHHHHHHHHHHHfffffffffffffffffffffffffffffff', data)

            data_enum.append(data_org[telem_value-1])  
            #pitch.append(data_org[1])
            
            data_enum = data_enum[-x_len:]
            #pitch = pitch[-x_len:]
            
            linex.set_ydata(data_enum)
            #liney.set_ydata(pitch)
            return linex,
        except:
            print(data)
try:  
    ani = animation.FuncAnimation(fig, animate, fargs=(data_enum,), interval=0.05, blit=True)
    plt.show()
except:
    print("No plot")