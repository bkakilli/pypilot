# Drone controller package
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017

import time, signal, sys, math, thread, argparse, imp, logging

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
#from bluetooth import *
from struct import *

from periodicrun import periodicrun

class Controller:

    def __init__(self, lggr, cfg):

        self.logger = lggr

        self.vehicle = None
        self.stopProgram = False
        self.globPose = (0,0,0,0,0,0,0)
        self.homePose_xyz = [0,0,0]
        self.homeSet = False

        self.cfg = cfg

        self.speed = 0.01
        self.targetV = [0,0,0]

    def connectToVehicle(self):
        logger.info('Connecting to vehicle on: %s' % self.cfg['port'])
        vehicle = connect(self.cfg['port'], baud=self.cfg['baud'], wait_ready=self.cfg['wait_ready'])
        logger.info('Connected to the vehice on %s', self.cfg['port'])

    def guidance():
        print 'test'

    def arm_and_takeoff_nogps(aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6

        logger.info("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check, just comment it with your own responsibility.
        while not vehicle.is_armable:
            logger.info(" Waiting for vehicle to initialise...")
            time.sleep(1)


        logger.info( "Arming motors")
        # Copter should arm in GUIDED_NOGPS mode
        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        vehicle.armed = True

        while not vehicle.armed:
            logger.info( " Waiting for arming...")
            time.sleep(1)

        print "Taking off!"

        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            logger.info( " Altitude: ", current_altitude)
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                logger.info( "Reached target altitude")
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

    def guidance(self):
        print 'test'

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
        ch.setLevel(logging.DEBUG)
        logger.debug('Verbose level is set 2. Everything will be displayed as much as possible.')

    controller = Controller(logger, cfg)
    #controller.connectToVehicle()

    pr = periodicrun(cfg['controllerPeriod'], controller.guidance, accuracy=0.01, lifetime=1)
    pr.run_thread()

    pr.join()
