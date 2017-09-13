import socket

UDP_IP = "128.230.68.234"
UDP_PORT = 52031

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(256) # buffer size is 1024 bytes
    print "received message:", data