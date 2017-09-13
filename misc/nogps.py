#!/usr/bin/env python

"""
todo:
vehicle init position problem
loop exceeds execution period
"""

import time, signal, sys, math, thread

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
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



def xyz2ned(xyz):
	ned = [xyz[1],xyz[0],-xyz[2]]
	return ned

#def signal_handler():
#	stopProgram = True
#
#signal.signal(signal.SIGINT, signal_handler)


def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)


    print "Arming motors"
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print " Altitude: ", current_altitude
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print "Reached target altitude"
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)


def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """

    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """

    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                             0,
                                                             0,                                         #target system
                                                             0,                                         #target component
                                                             0b00000000,                                #type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),    #q
                                                             0,                                         #body roll rate in radian
                                                             0,                                         #body pitch rate in radian
                                                             math.radians(yaw_rate),                    #body yaw rate in radian
                                                             thrust)                                    #thrust
    vehicle.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(modf[0])

        # Send command to vehicle on 1 Hz cycle
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


#def actuatorLoop():

period = 0.05	# seconds

ang = 0
vx = False
vy = False
vz = False
setOnOffboard = True
def mainLoop():
	global ang, period, globPose, homeSet, targetV, speed, vx,vy,vz, setOnOffboard


	#n = vehicle.location.local_frame.north
	#e = vehicle.location.local_frame.east
	#d = vehicle.location.local_frame.down
	#print "Position: ", n, " ", e, " ", d

	if vehicle.mode == "GUIDED_NOGPS":

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
		print 'GUIDED_NOGPS mode is on.'

		targetV[0] = 0.0
		targetV[1] = 0.0
		targetV[2] = 0.0

	#print "Vehicle:   ", vehicle.location.local_frame
	print "Bluetooth: ", globPose
	#print "Difference:", vehicle.location.local_frame.north-globPose[0], ", ", vehicle.location.local_frame.east-globPose[1], ", ", vehicle.location.local_frame.down, ", ", globPose[2]
	#print ""

def btTestLoop():
	print "Global pose: ", globPose


# Start bt connection
# thread.start_new_thread(btConnection, ())

# Connect to the vehicle
connection_string = "/dev/ttyS0"
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, baud=115200, wait_ready=True)
print 'Connected to the vehice on ', connection_string

time.sleep(1);

# Setup periodic loop and start it
from periodicrun import periodicrun
pr = periodicrun(period, mainLoop, accuracy=0.01)
#ac = periodicrun(period, actuatorLoop, accuracy=0.001)

pr.run_thread()
#ac.run_thread()

while not stopProgram:
	choice = raw_input("Make your choice: ")
	if str(choice) == "q":
		pr.interrupt()
		#ac.interrupt()
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
#ac.join()

print "Closing vehicle connection."
vehicle.close()
