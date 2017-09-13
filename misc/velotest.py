#!/usr/bin/env python

"""
todo:
vehicle init position problem
loop exceeds execution period
"""

import time, signal, sys, math, thread

from dronekit import connect, VehicleMode
from pymavlink import mavutil
from bluetooth import *
from struct import *

vehicle = None
stopProgram = False
globPose = (0,0,0,0,0,0,0)
homePose_xyz = [0,0,0]
homeSet = False

speed = 0.01
targetV = [0,0,0]

def btConnection():
	global globPose, homeSet
	
	pcount = 0
	
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

	print("connected to tango.")

	while not stopProgram:
		received = sock.recv(28)
		#print received
		rPose = unpack('<fffffff', received)
		globPose = rPose
		
		# Capture first 100 pose data and set the home by taking the average
		if not homeSet:
			homePose_xyz[0] = homePose_xyz[0] + rPose[0]
			homePose_xyz[1] = homePose_xyz[1] + rPose[1]
			homePose_xyz[2] = homePose_xyz[2] + rPose[2]
			pcount = pcount + 1
			if pcount == 100:
				homeSet = True
				homePose_xyz[0] = homePose_xyz[0] / 100
				homePose_xyz[1] = homePose_xyz[1] / 100
				homePose_xyz[2] = homePose_xyz[2] / 100
				print "Home position set: ", homePose_xyz
	
	print("Closing bluetooth connection.")
	sock.close()

	
def send_ned_velocity_no_dur(velocity_x, velocity_y, velocity_z):
	"""
	Move vehicle in direction based on specified velocity vectors.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,	   # time_boot_ms (not used)
		0, 0,	# target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)	# yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
		
	vehicle.send_mavlink(msg)
	
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
	"""
	Move vehicle in direction based on specified velocity vectors.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,	   # time_boot_ms (not used)
		0, 0,	# target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)	# yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

	# send command to vehicle on 20 Hz cycle
	freq = 20
	for x in range(0,duration*freq):
		vehicle.send_mavlink(msg)
		time.sleep(1/freq)
	
def xyz2ned(xyz):
	ned = [xyz[1],xyz[0],-xyz[2]]
	return ned

#def signal_handler():
#	stopProgram = True
#
#signal.signal(signal.SIGINT, signal_handler)

def sendArbitraryTargets():
	for i in range(0,100):
		goto_position_target_local_ned([0,0,0])
		time.sleep(0.02)
		print i,".",

def switchToOffboard():
	sendArbitraryTargets()
	# Change the mode
	vehicle.mode = VehicleMode("OFFBOARD")
	while not vehicle.mode == "OFFBOARD":
		print "Switching to offboard mode..."
		time.sleep(1)

def actuatorLoop():
	global targetV
	"""
	Move vehicle in direction based on specified velocity vectors.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,	   # time_boot_ms (not used)
		0, 0,	# target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		targetV[0], targetV[1], targetV[2], # x, y, z velocity in m/s
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)	# yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
		
	vehicle.send_mavlink(msg)
	
period = 0.05	# seconds

ang = 0
count = 0
vx = False
vy = False
vz = False
setOnOffboard = True
def mainLoop():
	global count, ang, period, globPose, homeSet, targetV, speed, vx,vy,vz, setOnOffboard

	count = count+1
	
	#n = vehicle.location.local_frame.north
	#e = vehicle.location.local_frame.east
	#d = vehicle.location.local_frame.down
	#print "Position: ", n, " ", e, " ", d
	
	if vehicle.mode == "OFFBOARD":
					
		#send_ned_velocity(0.0,-0.2,0.0,2)
		#if vx:
		#	send_ned_velocity(0.2,0.0,0.0,2)
		#	vx = False
		#elif vy:
		#	send_ned_velocity(0.0,0.2,0.0,2)
		#	vy = False
		#elif vz:
		#	send_ned_velocity(0.0,0.0,0.2,2)
		#	vz = False
		#else:
		#	send_ned_velocity_no_dur(0,0,0)
			
		## go to home
		#target = [0,0,0]
		#
		#d = target
		#d[0] = d[0] - globPose[0]
		#d[1] = d[1] - globPose[1]
		#d[2] = d[2] - globPose[2]
		## d is the distance vector to target
		#
		#v = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) / speed
		#targetV[0] = v * globPose[0]
		#targetV[1] = v * globPose[1]
		#targetV[2] = v * globPose[2]
		
		targetV[0] = 0.0
		targetV[1] = 0.0
		targetV[2] = 0.0

	else:
		#print "Waiting to switch offboard..."
		setOnOffboard = True
		send_ned_velocity(0,0,0,1)

	#print "Vehicle:   ", vehicle.location.local_frame
	print "Bluetooth: ", globPose
	#print "Difference:", vehicle.location.local_frame.north-globPose[0], ", ", vehicle.location.local_frame.east-globPose[1], ", ", vehicle.location.local_frame.down, ", ", globPose[2]
	#print ""
		
def btTestLoop():
	print "Global pose: ", globPose
	

# Start bt connection
thread.start_new_thread(btConnection, ())

# Connect to the vehicle
connection_string = "/dev/ttyS0"
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, baud=921600, wait_ready=False)
print 'Connected to the vehice on ', connection_string

time.sleep(1);

# Setup periodic loop and start it
from periodicrun import periodicrun
pr = periodicrun(period, mainLoop, accuracy=0.01)
ac = periodicrun(period, actuatorLoop, accuracy=0.001)

pr.run_thread()
ac.run_thread()

while not stopProgram:
	choice = raw_input("Make your choice: ")
	if str(choice) == "q":
		pr.interrupt()
		ac.interrupt()
		stopProgram = True
		break
	elif str(choice) == "0":
		testPose = [0,0,0]
		#setVehiclePose(testPose[0],testPose[1],testPose[2],0,0,0)
		
	elif str(choice) == "1":
		testPose = [10,0,0]
		#setVehiclePose(testPose[0],testPose[1],testPose[2],0,0,0)
		
	elif str(choice) == "2":
		testPose = [10,10,0]
		#setVehiclePose(testPose[0],testPose[1],testPose[2],0,0,0)
		
	elif str(choice) == "o":
		vehicle.mode = VehicleMode("OFFBOARD")
		
	elif str(choice) == "m":
		vehicle.mode = VehicleMode("STABILIZE")
		
	elif str(choice) == "vx":
		vx = True
		
	elif str(choice) == "vy":
		vy = True
		
	elif str(choice) == "vz":
		vz = True
		

pr.join()
ac.join()

print "Closing vehicle connection."
vehicle.close()
