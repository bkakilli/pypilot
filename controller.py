# Drone controller package
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017
#
# ToDo:
# Thread based manual command system
# Curses support

import time, signal, sys, math, thread, argparse, imp, logging
import struct, threading

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

from modules.periodicrun import periodicrun
from modules.estimator import ViconTrackerEstimator as Estimator
from modules.actuator import SimpleVelocityController as Actuator
from modules.mission import Mission
from modules.mission import Task

class Controller(object):

    def __init__(self, cfg):

        self.stopProgram = False
        self.cfg = cfg

        self.vehicle = None
        self.homePose = [0,0,0,0,0,0]
        self.homeSet = False
        self.mission = None

        self.actuator = None
        self.estimator = None
        self.actuatorTarget = [0,0,2]

        self.followObjPos = [0,0,0]

        self.armingInProgress = False

    def connectToVehicle(self):
        logger.info('Connecting to vehicle on: %s' % self.cfg['port'])
        self.vehicle = connect(self.cfg['port'], baud=self.cfg['baud'], wait_ready=self.cfg['wait_ready'])
        logger.info('Connected to the vehice on %s', self.cfg['port'])

    '''
    def positionReceiver(self):
        print 'Position receiver thread started.'
        while not self.stopProgram:
            data, addr = self.udpsock.recvfrom(256) # buffer size is 1024 bytes
            obj1 = data[8]
            obj2 = data[83]

            if obj1 == 'B':
                self.globPose =    [struct.unpack('d',data[32:40])[0]/1000,
                                    struct.unpack('d',data[40:48])[0]/1000,
                                    struct.unpack('d',data[48:56])[0]/1000,
                                    struct.unpack('d',data[56:64])[0],
                                    struct.unpack('d',data[64:72])[0],
                                    struct.unpack('d',data[72:80])[0]]
                if obj2 != 0:
                    self.followObjPos = [struct.unpack('d',data[107:115])[0]/1000,
                                         struct.unpack('d',data[115:123])[0]/1000,
                                         3]
                else:
                    self.followObjPos = [0,0,2]

            elif obj2 == 'B':
                self.globPose =    [struct.unpack('d',data[107:115])[0]/1000,
                                    struct.unpack('d',data[115:123])[0]/1000,
                                    struct.unpack('d',data[123:131])[0]/1000,
                                    struct.unpack('d',data[131:139])[0],
                                    struct.unpack('d',data[139:147])[0],
                                    struct.unpack('d',data[147:155])[0]
                                    ]
                self.followObjPos = [struct.unpack('d',data[32:40])[0]/1000,
                                     struct.unpack('d',data[40:48])[0]/1000,
                                     3]
    '''

    def arm_vehicle(self):
        """
        Arms vehicle
        """
        self.armingInProgress = True

        logger.debug("Check if armed for take-off.")
        if not self.vehicle.armed:

            logger.info("Switching to STABILIZE mode before arming.")
            while not self.vehicle.mode == "STABILIZE":
                self.vehicle.mode = VehicleMode("STABILIZE")
                time.sleep(1)
            logger.info( "Waiting for arming...")
            while not self.vehicle.armed:
                self.vehicle.armed = True
                time.sleep(1)

        logger.info("Switching to GUIDED_NOGPS mode.")
        while not self.vehicle.mode == "GUIDED_NOGPS":
            self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
            time.sleep(1)

        self.armingInProgress = False


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

    def distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

    def setVehicleMode(self, mode):
        self.vehicle.mode = VehicleMode(mode)

    def run(self):

        cfg = self.cfg

        self.stopProgram = False
        #self.udpsock = socket.socket(socket.AF_INET, # Internet
        #                     socket.SOCK_DGRAM) # UDP
        #self.udpsock.bind((self.cfg['UDP_IP'], self.cfg['UDP_PORT']))

        self.connectToVehicle()
        self.actuator = Actuator(cfg['tuning']['PID_xy'], cfg['tuning']['PID_z'])
        self.estimator = Estimator(cfg['UDP_IP'], cfg['UDP_PORT'], cfg['VICON_DRONENAME'])
        self.estimator.run()

        self.poscapturethread = threading.Thread(target=self.positionReceiver)
        self.cthread = periodicrun(cfg['guidancePeriod'], self.guidanceLoop, accuracy=0.001)
        self.athread = periodicrun(cfg['actuatorPeriod'], self.actuatorLoop, accuracy=0.001)
        self.poscapturethread.start()
        self.cthread.run_thread()
        self.athread.run_thread()

        self.monitor()

    def stop(self):
        logger.debug('Controller shut down.')
        self.stopProgram = True
        self.athread.interrupt()
        self.cthread.interrupt()
        self.vehicle.close()


    def join(self):
        self.poscapturethread.join()
        self.athread.join()
        self.cthread.join()

    def guidanceLoop(self):

        if self.mission:
            globPose = estimator.getPoses(0)
            task = self.mission.tasks[0]

            if   task.type == Task.TYPE.TAKEOFF:
                if not task.active:
                    task.start()
                if not self.armingInProgress:
                    self.arm_vehicle()
                    self.actuatorTarget = [globPose[0], globPose[1], task.target[2]]

                # Check completed
                if self.distance(globPose[:3], task.target) < 0.1: #TARGETREACHPRECISION
                    self.mission.remove(0)

            elif task.type == Task.TYPE.LAND:
                if not task.active:
                    task.start()
                setVehicleMode('LAND')

                # Check completed
                if self.vehicle.mode == 'LAND':
                    self.mission.remove(0)

            elif task.type == Task.TYPE.HOVER:
                if not task.active:
                    task.start(target=self.mission.previousTarget[:3])
                self.actuatorTarget = task.target

                # Check completed
                if task.istimeout():
                    self.mission.remove(0)

            elif task.type == Task.TYPE.MOVETO:
                if not task.active:
                    if task.relative:
                        prevTarget = self.mission.previousTarget
                        task.target =   [prevTarget[0]+task.target[0],
                                         prevTarget[1]+task.target[1],
                                         prevTarget[2]+task.target[2]]

                    task.start()
                self.actuatorTarget = task.target

                # Check completed
                if self.distance(globPose[:3], task.target) < 0.1: #TARGETREACHPRECISION
                    self.mission.remove(0)

            elif task.type == Task.TYPE.FOLLOW:
                if not task.active:
                    task.start()
                task.target = self.followObjPos
                self.actuatorTarget = task.target

                # Check completed
                if task.istimeout():
                    self.mission.remove(0)


    # This actuator loop is executed in every cfg['actuatorPeriod'] sec
    def actuatorLoop(self):

        # Get current control values
        c = self.actuator.step(estimator.getPoses(0), self.actuatorTarget)

        roll_angle  = c['roll_angle']
        pitch_angle = c['pitch_angle']
        thrust = c['thrust']
        yaw_rate = c['yaw_rate']

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


    def monitor(self):
        while True:
            choice = raw_input("Make your choice: ")
            if str(choice) == "q":
                break

            elif str(choice) == "gn":
                self.setVehicleMode('GUIDED_NOGPS')

            elif str(choice) == "m":
                self.setVehicleMode('STABILIZE')

            elif str(choice) == "mi":
                mission = Mission()

                task = Task(Task.TYPE.TAKEOFF)
                task.target = [0,0,2]
                mission.appendTask(task)

                task = Task(Task.TYPE.HOVER)
                task.duration = 3
                mission.appendTask(task)

                task = Task(Task.TYPE.LAND)
                mission.appendTask(task)

                self.mission = mission

        self.stop()

#######################  Main Program  ##########################

def test(cfg):
    pass

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

    #test(cfg)
    controller = Controller(cfg)
    controller.run()
    controller.join()
