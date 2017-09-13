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
from bluetooth import *
from struct import *

vehicle = None
stopProgram = False
globPose = (0,0,0,0,0,0,0)
homePose_xyz = [0,0,0]
homeSet = False

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
	sock.close()

def setVehiclePose(x, y, z, roll, pitch, yaw):
	'''
	usec 		: Timestamp (microseconds, synced to UNIX time or since system boot) (uint64_t)
	x    		: Global X position (float)
	y    		: Global Y position (float)
	z    		: Global Z position (float)
	roll 		: Roll angle in rad (float)
	pitch		: Pitch angle in rad (float)
	yaw  		: Yaw angle in rad (float)
	'''
	usec = int(round(time.time() * 1000))
	msg = vehicle.message_factory.vision_position_estimate_encode(
		usec,
		x, y, z,
		roll, pitch, yaw);
	# send command to vehicle
	vehicle.send_mavlink(msg)

def setVehiclePose2(x, y, z):
	'''
	usec 		: Timestamp (microseconds, synced to UNIX time or since system boot) (uint64_t)
	x    		: Global X position (float)
	y    		: Global Y position (float)
	z    		: Global Z position (float)
	roll 		: Roll angle in rad (float)
	pitch		: Pitch angle in rad (float)
	yaw  		: Yaw angle in rad (float)
	'''
	usec = int(round(time.time() * 1000))
	msg = vehicle.message_factory.att_pos_mocap_encode(
		usec,
		x, y, z);
	# send command to vehicle
	vehicle.send_mavlink(msg)

def goto_position_target_local_ned(ned):
	"""
	Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
	location in the North, East, Down frame.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,	   # time_boot_ms (not used)
		0, 0,	# target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111111000, # type_mask (only positions enabled)
		ned[0], ned[1], ned[2],
		0, 0, 0, # x, y, z velocity in m/s  (not used)
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)	# yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	# send command to vehicle
	vehicle.send_mavlink(msg)

def set_attitude_target(quat):
    msg = vehicle.message_factory.set_attitude_target_encode(
    	0,
    	0,                #target system
    	0,                #target component
    	0b11100010,       #type mask
    	quat,			  #q
    	0,                #body roll rate
    	0,                #body pitch rate
    	0,                #body yaw rate
    	0)                #thrust
	# send command to vehicle
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

def makeYaw(angle):
	w = 1.0
	x = 0.0
	y = 0.0
	z = 0.0

	#yaw motion
	rad = angle * math.pi/180;
	w = math.cos(rad/2)
	z = math.sin(rad/2)
	
	q = [w, x, y, z]
	set_attitude_target(q)

	
	
period = 0.05	# seconds

ang = 0
count = 0
def mainLoop():
	if homeSet:
		global count, ang, period, globPose, homeSet
		count = count+1
		setVehiclePose(globPose[0],globPose[1],globPose[2],0,0,0)
		
		#n = vehicle.location.local_frame.north
		#e = vehicle.location.local_frame.east
		#d = vehicle.location.local_frame.down
		#print "Position: ", n, " ", e, " ", d
		
		if vehicle.mode == "OFFBOARD":
			
			# go to home
			goto_position_target_local_ned( xyz2ned(homePose_xyz) )
			
			#if count%(5/period) == 0:	# every N seconds
			#	goto_position_target_local_ned(0,0,0)
			#	pass
			#	ang = -ang
			#	makeYaw(ang)
			#	count = 0 
			# makeYaw(0)
			#else:
			#	goto_position_target_local_ned(0,0,0)

		else:
			#print "Waiting to switch offboard..."
			goto_position_target_local_ned([0,0,0])

		print "Vehicle:   ", vehicle.location.local_frame
		print "Bluetooth: ", globPose
		print "Difference:", vehicle.location.local_frame.north-globPose[0], ", ", vehicle.location.local_frame.east-globPose[1], ", ", vehicle.location.local_frame.down, ", ", globPose[2]
		print ""
		
def btTestLoop():
	print "Global pose: ", globPose
	
testPose = [0,0,0]
testPoseCount = 0
def posEstTest():
	global testPose, testPoseCount
	if testPoseCount == 50:
		testPoseCount = 0
		print vehicle.location.local_frame
	testPoseCount = testPoseCount+1
	setVehiclePose2(testPose[0],testPose[1],testPose[2])
	goto_position_target_local_ned([0,0,0])
	

# Connect to the vehicle
connection_string = "/dev/ttyS0"
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, baud=921600, wait_ready=False)
print 'Connected to the vehice on ', connection_string

time.sleep(1);
#print_vehicle_state()

#while not vehicle.armed:
#	print "Vehicle not armed"
#	vehicle.close()
#	sys.exit(0)
		
# Start bt connection
#thread.start_new_thread(btConnection, ())

# Send some initial pose data
for i in range (0,100):
	setVehiclePose(0,0,0,0,0,0)
	time.sleep(0.02)

# Setup periodic loop and start it
from periodicrun import periodicrun
pr = periodicrun(period, posEstTest, accuracy=0.01)

pr.run_thread()

while not stopProgram:
	choice = raw_input("Make your choice: ")
	if str(choice) == "q":
		pr.interrupt()
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
		vehicle.mode = VehicleMode("MANUAL")
		

pr.join()

print "Closing vehicle connection."
vehicle.close()
