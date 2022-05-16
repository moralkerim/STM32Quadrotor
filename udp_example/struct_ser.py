import struct
import sys

with open('LOG.txt','rb') as f:
    line = f.read(112)
    while 1:
        line = f.read(112)
        print(line)
        print(sys.getsizeof(line))
        data_org = struct.unpack('<ffffffffffffHHHHfffffffffffffL', line)
        print(data_org)



