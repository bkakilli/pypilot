from bluetooth import *
from struct import *

addr = "00:16:D4:F3:59:70"
uuid = "00001101-0000-1000-8000-00805F9B34FB"

service_matches = find_service( uuid = uuid, address = addr )

if len(service_matches) == 0:
	print("couldn't find the SampleServer service =(")
	sys.exit(0)

first_match = service_matches[0]
port = first_match["port"]
name = first_match["name"]
host = first_match["host"]

print("connecting to \"%s\" on %s" % (name, host))

# Create the client socket
sock=BluetoothSocket( RFCOMM )
sock.connect((host, port))

print("connected.  type stuff")

while True:
	received = sock.recv(28)
	#print received
	a = unpack('<fffffff', received)
	print "received", a

	


sock.close()