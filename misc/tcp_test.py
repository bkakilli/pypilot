import socket, struct

TCP_IP = "10.1.15.2"
TCP_PORT = 9876
BUFFER_SIZE = 256

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_STREAM) # TCP
sock.connect((TCP_IP, TCP_PORT))

while True:
    data = sock.recvfrom(BUFFER_SIZE) # buffer size is 1024 bytes
    print "received message len:", len(data)
    num = struct.unpack('i',data[:4])[0]
    print "num: ", num
    trans = [struct.unpack('d',data[32:40])[0], struct.unpack('d',data[40:48])[0], struct.unpack('d',data[48:56])[0]]
    print "trans", trans
