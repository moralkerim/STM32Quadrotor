import struct
import sys
import socket
import select

port = 9000
ip = ''
mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#mySocket.setblocking(False)
mySocket.bind((ip, port))

def empty_socket(sock):
    """remove the data present on the socket"""
    input = [sock]
    while 1:
        inputready, o, e = select.select(input,[],[], 0.0)
        if len(inputready)==0: break
        for s in inputready: s.recv(1024)

while 1:
    (data, addr) = mySocket.recvfrom(1024)

    empty_socket(mySocket)
    #data_org = struct.unpack('<ffffffffffffHHHHfffffffffffffL', line)
    print(data)
    print("======================")



