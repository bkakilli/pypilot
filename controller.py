# Drone controller package
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017
#
# ToDo:
# Thread based manual command system
# Curses support

import time, signal, sys, math, thread, argparse, imp, logging
import socket, struct, threading

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
#from bluetooth import *

from periodicrun import periodicrun
import numpy as np

class Controller(object):

    def __init__(self, cfg):

        self.vehicle = None
        self.stopProgram = False
        self.globPose = np.array([0,0,0,0,0,0])
        self.homePose = np.array([0,0,0,0,0,0])
        self.homeSet = False

        self.cfg = cfg
        self.PID_xy = PID_xy = np.array(self.cfg['tuning']['PID_xy'])

        self.velocity = np.array([0,0,0])
        self.lasttime = time.time()
        self.lastPos = np.array([0,0,0,0,0,0])
        self.targetPos = np.array([0,0,2])
        self.error = np.zeros((3,3))

    def connectToVehicle(self):
        logger.info('Connecting to vehicle on: %s' % self.cfg['port'])
        self.vehicle = connect(self.cfg['port'], baud=self.cfg['baud'], wait_ready=self.cfg['wait_ready'])
        logger.info('Connected to the vehice on %s', self.cfg['port'])

    def positionReceiver(self):
        print 'Position receiver thread started.'
        while not self.stopProgram:
            data, addr = self.udpsock.recvfrom(256) # buffer size is 1024 bytes
            obj1 = data[8]
            obj2 = data[83]

            if obj1 == 'B':
                self.globPose = np.array(
                                   [struct.unpack('d',data[32:40])[0]/1000,
                                    struct.unpack('d',data[40:48])[0]/1000,
                                    struct.unpack('d',data[48:56])[0]/1000,
                                    struct.unpack('d',data[56:64])[0],
                                    struct.unpack('d',data[64:72])[0],
                                    struct.unpack('d',data[72:80])[0]
                                    ])
                if obj2 != 0:
                    self.targetPos = np.array(
                                       [struct.unpack('d',data[107:115])[0]/1000,
                                        struct.unpack('d',data[115:123])[0]/1000,
                                        3
                                        ])
                else:
                    self.targetPos = np.array([0,0,2])

            elif obj2 == 'B':
                self.globPose = np.array(
                                   [struct.unpack('d',data[107:115])[0]/1000,
                                    struct.unpack('d',data[115:123])[0]/1000,
                                    struct.unpack('d',data[123:131])[0]/1000,
                                    struct.unpack('d',data[131:139])[0],
                                    struct.unpack('d',data[139:147])[0],
                                    struct.unpack('d',data[147:155])[0]
                                    ])
                self.targetPos = np.array(
                                   [struct.unpack('d',data[32:40])[0]/1000,
                                    struct.unpack('d',data[40:48])[0]/1000,
                                    3
                                    ])

    def arm_and_takeoff_nogps(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6

        logger.info("Switch to STABILIZE mode fore arming.")
        while not self.vehicle.mode == "STABILIZE":
            self.vehicle.mode = VehicleMode("STABILIZE")
            time.sleep(1)

        #logger.info("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check, just comment it with your own responsibility.
        #while not self.vehicle.is_armable:
        #    logger.info(" Waiting for vehicle to initialise...")
        #    time.sleep(1)


        logger.info( "Arming motors")
        while not self.vehicle.armed:
            logger.info( " Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        while not self.vehicle.mode == "GUIDED_NOGPS":
            self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
            time.sleep(1)

        logger.info( "Taking off!")
        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            logger.info( " Altitude: %f", current_altitude)
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                logger.info( "Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            self.set_attitude(thrust = thrust)
            time.sleep(0.2)


    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
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
        msg = self.vehicle.message_factory.set_attitude_target_encode(
                     0,
                     0,                                         #target system
                     0,                                         #target component
                     0b00000000,                                #type mask: bit 1 is LSB
                     self.to_quaternion(roll_angle, pitch_angle),    #q
                     0,                                         #body roll rate in radian
                     0,                                         #body pitch rate in radian
                     math.radians(yaw_rate),                    #body yaw rate in radian
                     thrust)                                    #thrust
        self.vehicle.send_mavlink(msg)

        if duration != 0:
            # Divide the duration into the frational and integer parts
            modf = math.modf(duration)

            # Sleep for the fractional part
            time.sleep(modf[0])

            # Send command to vehicle on 1 Hz cycle
            for x in range(0,int(modf[1])):
                time.sleep(1)
                self.vehicle.send_mavlink(msg)

    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
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

    def setVehicleMode(self, mode):
        self.vehicle.mode = VehicleMode(mode)

    def send_attitude(self, att):
        print atts

    def run(self):

        self.stopProgram = False
        self.udpsock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.udpsock.bind((self.cfg['UDP_IP'], self.cfg['UDP_PORT']))

        self.connectToVehicle()

        self.poscapturethread = threading.Thread(target=self.positionReceiver)
        self.cthread = periodicrun(cfg['guidancePeriod'], self.guidance, accuracy=0.001)
        self.athread = periodicrun(cfg['actuatorPeriod'], self.actuator, accuracy=0.001)
        self.poscapturethread.start()
        self.cthread.run_thread()
        self.athread.run_thread()

        self.monitor()

    def stop(self):
        logger.debug('Controller shut down.')
        self.vehicle.close()
        self.stopProgram = True
        self.athread.interrupt()
        self.cthread.interrupt()


    def join(self):
        self.poscapturethread.join()
        self.athread.join()
        self.cthread.join()

    def guidance(self):
        logger.debug('Guidance loop started.')
        print 'Glob Position: ', self.globPose
        print 'Target Position: ', self.targetPos
        print 'velocity: ', self.velocity

    # This actuator loop is executed in every cfg['actuatorPeriod'] sec
    def actuator(self):
        currPos = self.globPose
        currTime = time.time()
        # Calculate the velocity
        self.velocity = (currPos[:3] - self.lastPos[:3]) / (currTime - self.lasttime)

        self.lasttime = currTime
        self.lastPos = currPos

        # Create a roll pitch angle from PID controller
        PID_xy = self.PID_xy #np.array(self.cfg['tuning']['PID_xy'])
        PID_z  = np.array(self.cfg['tuning']['PID_z'])

        gPos = currPos[:3]
        theta = -currPos[5]    # yaw correction in radians
        rotZ = np.array(
                        [[ np.cos(theta), -np.sin(theta), 0],
                         [ np.sin(theta),  np.cos(theta), 0],
                         [ 0            ,  0            , 1]]
                        )

        # Calculate the 3D error vector with corrected rotation
        distToTarget = rotZ.dot(self.targetPos - gPos)

        # Set the desired speed based on the predefined pattern
        desiredVelocity = 1 / (1 + np.exp(-5*np.absolute(distToTarget)+5))
        desiredVelocity = desiredVelocity * distToTarget / np.absolute(distToTarget)

        p_e = desiredVelocity - self.velocity
        p_e[2] = distToTarget[2]

        # Update the error matrix
        err = self.error
        err = np.array([
                        [p_e[0], err[0,1]+p_e[0], p_e[0]-err[0,0]],
                        [p_e[1], err[1,1]+p_e[1], p_e[1]-err[1,0]],
                        [p_e[2], err[2,1]+p_e[2], p_e[2]-err[2,0]]
        ])
        self.error = err

        # Generate and set the target angle for XY plane
        angle = err[:2,:].dot(PID_xy)
        thrust_add = err[2,:].dot(PID_z)

        #gPos = currPos[:3]
        #theta = -currPos[5]    # yaw correction in radians
        #rotZ = np.array(
        #                [[ np.cos(theta), -np.sin(theta), 0],
        #                 [ np.sin(theta),  np.cos(theta), 0],
        #                 [ 0            ,  0            , 1]]
        #                )
        #
        ## Calculate the 3D error vector with corrected rotation
        #p_e = rotZ.dot(self.targetPos - gPos)
        #
        ## Update the error matrix
        #err = self.error
        #err = np.array([
        #                [p_e[0], err[0,1]+p_e[0], p_e[0]-err[0,0]],
        #                [p_e[1], err[1,1]+p_e[1], p_e[1]-err[1,0]],
        #                [p_e[2], err[2,1]+p_e[2], p_e[2]-err[2,0]]
        #])
        #self.error = err
        #
        ## Generate and set the target angle for XY plane
        #angle = err[:2,:].dot(PID_xy)
        #thrust_add = err[2,:].dot(PID_z)

        roll_angle  = angle[0]
        pitch_angle = -angle[1]
        thrust = 0.5 + thrust_add

        # Non-linear cutoff for safety
        if self.cfg['tuning']['NONLIN_SAFETY']:
            if roll_angle  > self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = self.cfg['tuning']['MAX_SAFE_ANGLE']
            if pitch_angle > self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = self.cfg['tuning']['MAX_SAFE_ANGLE']
            if roll_angle  < -self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = -self.cfg['tuning']['MAX_SAFE_ANGLE']
            if pitch_angle < -self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = -self.cfg['tuning']['MAX_SAFE_ANGLE']

        # Use defaults for the others
        yaw_rate = 0.0

        msg = self.vehicle.message_factory.set_attitude_target_encode(
                     0,
                     0,                                         #target system
                     0,                                         #target component
                     0b00000000,                                #type mask: bit 1 is LSB
                     self.to_quaternion(roll_angle, pitch_angle),    #q
                     0,                                         #body roll rate in radian
                     0,                                         #body pitch rate in radian
                     math.radians(yaw_rate),                    #body yaw rate in radian
                     thrust)                                    #thrust
        self.vehicle.send_mavlink(msg)


    #manual = {
    #0: send_attitude,
    #1: setVehicleMode,
    #2: arm_and_takeoff_nogps
    #}
    #
    #def manualCommand(self, comm):
    #    print 'Received command: ', comm;
    #    if 'val' in comm:
    #        self.manual[comm['type']](comm['val'])
    #    else:
    #        self.manual[comm['type']]

    def monitor(self):
        while True:
            choice = raw_input("Make your choice: ")
            if str(choice) == "q":
                break

            elif str(choice) == "gn":
                self.setVehicleMode('GUIDED_NOGPS')

            elif str(choice) == "m":
                self.setVehicleMode('STABILIZE')

            elif str(choice) == "t":
                self.arm_and_takeoff_nogps(self.cfg['takeoff_altitude'])

            elif str(choice) == "tl":
                self.arm_and_takeoff_nogps(0.5)
                self.setVehicleMode('LAND')

            elif str(choice) == "l":
                self.setVehicleMode('LAND')

            elif str(choice) == "u":
                self.set_attitude(yaw_rate = 10, duration=1)

            elif str(choice) == "y":
                self.set_attitude(yaw_rate = -10, duration=1)

            elif str(choice) == "w":
                self.set_attitude(pitch_angle = -5, duration=0.5)

            elif str(choice) == "s":
                self.set_attitude(pitch_angle = 5, duration=0.5)

            elif str(choice) == "a":
                self.set_attitude(roll_angle = -5, duration=0.5)

            elif str(choice) == "d":
                self.set_attitude(roll_angle = 5, duration=0.5)

            elif str(choice) == "2":
                self.PID_xy[0] = self.PID_xy[0] + 0.5

            elif str(choice) == "1":
                if self.PID_xy[2] >= 1:
                    self.PID_xy[0] = self.PID_xy[0] - 0.5

            #elif str(choice) == "4":
            #    self.PID_xy[1] = self.PID_xy[1] + 0.5
#
            #elif str(choice) == "3":
            #    if self.PID_xy[2] >= 1:
            #        self.PID_xy[1] = self.PID_xy[1] - 0.5

            elif str(choice) == "6":
                self.PID_xy[2] = self.PID_xy[2] + 0.5

            elif str(choice) == "5":
                if self.PID_xy[2] >= 1:
                    self.PID_xy[2] = self.PID_xy[2] - 0.5

        self.stop()

#######################  Main Program  ##########################


if __name__ == '__main__':

    # start logger
    logger = logging.getLogger('Controller')
    logger.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    formatter = logging.Formatter('%(name)s: %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    # arg parse here
    config_path = 'config.py'

    config_module = imp.load_source('config', config_path)
    cfg = config_module.cfg

    # Logging adjustment
    if cfg['verbose'] == 1:
        logger.setLevel(logging.INFO)
    elif cfg['verbose'] == 2:
        formatter = logging.Formatter('%(name)s | %(asctime)s: %(message)s')
        ch.setFormatter(formatter)
        logger.setLevel(logging.DEBUG)
        logger.debug('Verbose level is set 2. Everything will be displayed as much as possible.')
        print 'test'

    controller = Controller(cfg)
    controller.run()
    controller.join()
