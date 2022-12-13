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
      P_roll=27
      I_roll=28
      D_roll=29
      pid_roll=30
      P_pitch=31
      I_pitch=32
      D_pitch=33
      pid_pitch=34
      sonar_alt=35
      alt_thr = 36
      xb = 37
      yb = 38
      zb = 39
      vx = 40
      vy = 41
      vz = 42
      time_millis=43
      detected=44
      xcam = 45
      ycam = 46
      zcam = 47
      yawcam = 48
      accX = 49
      accY = 50
      accZ = 51
      magX = 52
      magY = 53
      magZ = 54
      lat = 55
      lon = 56
      alt_gps = 57
      xned = 58
      yned = 59
      zned = 60
      xbody_gps = 61
      ybody_gps = 62
      zbody_gps = 63
      vxbody_gps = 64
      vybody_gps = 65
      vzbody_gps = 66
      ch1=67
      ch2=68
      ch3=69
      ch4=70
      ch5=71
      ch6=72
      ch7=73
      ch8=74
      ch9=75
      ch10=76
      ch11=77
      pwm2_1 = 78
      pwm2_2 = 79
      pwm2_3 = 80
      pwm2_4 = 81
      P_yaw=82
      I_yaw=83
      D_yaw=84
      pid_yaw=85
      
      S_roll_11=86
      S_roll_12=87
      S_roll_13=88
      S_roll_21=89
      S_roll_22=90
      S_roll_23=91
      S_roll_31=92
      S_roll_32=93
      S_roll_33=94
      
      S_pitch_11=95
      S_pitch_12=96
      S_pitch_13=97
      S_pitch_21=98
      S_pitch_22=99
      S_pitch_23=100
      S_pitch_31=101
      S_pitch_32=102
      S_pitch_33=103
      
      S_yaw_11=104
      S_yaw_12=105
      S_yaw_13=106
      S_yaw_21=107
      S_yaw_22=108
      S_yaw_23=109
      S_yaw_31=110
      S_yaw_32=111
      S_yaw_33=112
      
      bno_roll=113
      bno_pitch=114
      bno_yaw=115
      bno_roll_rate=116
      bno_pitch_rate=117
      bno_yaw_rate=118

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
            data_org = struct.unpack('<ffffffffffffHHHHffffffffffffffffffffffffffLBhhhhfffhhhffffffffffffHHHHHHHHHHHHHHHfffffffffffffffffffffffffffffffffffff', data)

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