import struct
import sys

with open('LOG.txt','rb') as f:
    line = f.read(108)
    while 1:
        line = f.read(108)
        print(line)
        print(sys.getsizeof(line))
        data_org = struct.unpack('<ffffffffffffHHHHfffffffffffff', line)
        print(data_org)



