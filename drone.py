from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time, signal, sys, math

vehicle = None
stopProgram = False

def print_vehicle_state():
	print "\nGet all vehicle attribute values:"
	print " Autopilot Firmware version: %s" % vehicle.version
	print "   Major version number: %s" % vehicle.version.major
	print "   Minor version number: %s" % vehicle.version.minor
	print "   Patch version number: %s" % vehicle.version.patch
	print "   Release type: %s" % vehicle.version.release_type()
	print "   Release version: %s" % vehicle.version.release_version()
	print "   Stable release?: %s" % vehicle.version.is_stable()
	print " Autopilot capabilities"
	print "   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float
	print "   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float
	print "   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int
	print "   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int
	print "   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union
	print "   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp
	print "   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target
	print "   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned
	print "   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int
	print "   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain
	print "   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target
	print "   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination
	print "   Supports mission_float message type: %s" % vehicle.capabilities.mission_float
	print "   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration
	print " Global Location: %s" % vehicle.location.global_frame
	print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
	print " Local Location: %s" % vehicle.location.local_frame
	print " Attitude: %s" % vehicle.attitude
	print " Velocity: %s" % vehicle.velocity
	print " GPS: %s" % vehicle.gps_0
	print " Gimbal status: %s" % vehicle.gimbal
	print " Battery: %s" % vehicle.battery
	print " EKF OK?: %s" % vehicle.ekf_ok
	print " Last Heartbeat: %s" % vehicle.last_heartbeat
	print " Rangefinder: %s" % vehicle.rangefinder
	print " Rangefinder distance: %s" % vehicle.rangefinder.distance
	print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
	print " Heading: %s" % vehicle.heading
	print " Is Armable?: %s" % vehicle.is_armable
	print " System status: %s" % vehicle.system_status.state
	print " Groundspeed: %s" % vehicle.groundspeed	# settable
	print " Airspeed: %s" % vehicle.airspeed	# settable
	print " Mode: %s" % vehicle.mode.name	# settable
	print " Armed: %s" % vehicle.armed	# settable

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

def goto_position_target_local_ned(north, east, down):
	"""
	Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
	location in the North, East, Down frame.
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,	   # time_boot_ms (not used)
		0, 0,	# target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111111000, # type_mask (only positions enabled)
		north, east, down,
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
    veh1.send_mavlink(msg)

def signal_handler():
	stopProgram = True

signal.signal(signal.SIGINT, signal_handler)

connection_string = "/dev/ttyS0"
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, baud=921600, wait_ready=True)

print_vehicle_state()

#while not vehicle.armed:
#	print "Vehicle not armed"
#	vehicle.close()
#	sys.exit(0)

def sendArbitraryTargets():
	for i in range(0,100):
		goto_position_target_local_ned(0,0,0)
		time.sleep(0.02)
		print i,".",

def switchToOffboard():
	sendArbitraryTargets()
	# Change the mode
	vehicle.mode = VehicleMode("OFFBOARD")
	while not vehicle.mode == "OFFBOARD":
		print "Switching to offboard mode..."
		time.sleep(1)

#sendArbitraryTargets()
#switchToOffboard()
'''
while (not stopProgram):# and vehicle.mode == "OFFBOARD":
	print "Local frame : ", vehicle.location.local_frame
	setVehiclePose(0,0,0,0,0,0)
	goto_position_target_local_ned(50,50,-10)
	time.sleep(0.2)
'''

for i in range (0,100):
	setVehiclePose(0,0,0,0,0,0)
	time.sleep(0.02)

def makeYaw(angle):
	w = 1.0
	x = 0.0
	y = 0.0
	z = 0.0

	#yaw motion
	rad = angle * pi/180;
	w = cos(rad/2)
	z = sin(rad/2)

	set_attitude_target([w x y z])

count = 0
ang = 2.0
period = 0.01	# seconds

def mainLoop():

	count = count+1
	n = vehicle.location.local_frame.north
	e = vehicle.location.local_frame.east
	d = vehicle.location.local_frame.down

	if vehicle.mode == "OFFBOARD":
		#goto_position_target_local_ned(n,e,d)

		if count%(5*1/period) == 0:	# every N seconds
			ang = -ang
			makeYaw(ang)
			count = 0

	else:
		print "Waiting to switch offboard..."
		goto_position_target_local_ned(n,e,d)

	print "Position: ", vehicle.location.local_frame


from periodicrun import periodicrun
pr = periodicrun(period, mainLoop, accuracy=0.001)

pr.run_thread()

while True:
	choice = input("Make your choice: ")
	if choice == "q":
		pr.interrupt()
		break

pr.join()

print "Closing vehicle connection."
vehicle.close()
