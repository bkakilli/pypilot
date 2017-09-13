import socket, struct

UDP_IP = "0.0.0.0"
UDP_PORT = 9876

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(256) # buffer size is 1024 bytes
    print "received message len:", len(data)
    num = struct.unpack('i',data[:4])[0]
    print "num: ", num
    trans = [struct.unpack('d',data[32:40])[0], struct.unpack('d',data[40:48])[0], struct.unpack('d',data[48:56])[0]]
    print "trans", trans
