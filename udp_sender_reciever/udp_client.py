import sys
import socket
import numpy as np
from time import sleep


# print 'Number of arguments:', len(sys.argv), 'arguments.'
# print 'Argument List:', str(sys.argv)
# print 'Usage: udp_client.py IP PORT DATA_SIZE REPETITION_IN_S'
# print 'Example: udp_client.py 192.168.1.55 22019 900 0.02'

# if len(sys.argv) < 5:
#     print("Enter valid data")
#     exit()
#
# ip= sys.argv[1]
# port=sys.argv[2]
# bufferSize=sys.argv[3]
# repetition=sys.argv[4]

ip= "192.168.1.183"
port="64109"
bufferSize="1024"
repetition="0.02"

msgFromClient = np.ones(int(bufferSize) , dtype=np.int8)
bytearray(msgFromClient)

serverAddressPort = (ip, int(port))

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Send to server using created UDP socket
while True:
    UDPClientSocket.sendto(msgFromClient, serverAddressPort)
    sleep(float(repetition))

