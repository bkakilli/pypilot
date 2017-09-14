# Drone controller package
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017
#
# ToDo:
# Thread based manual command system
# Curses support

import time, math, argparse, imp, logging

from dronekit import connect, VehicleMode

from modules.periodicrun import periodicrun
from modules.estimator import ViconTrackerEstimator as Estimator
from modules.actuator import SimpleVelocityController as Actuator
from modules.mission import Mission
from modules.mission import Task

class Controller(object):

    def __init__(self, cfg):

        self.cfg = cfg

        self.vehicle = None
        self.homePose = [0,0,0,0,0,0]
        self.vehiclePose = [0,0,0,0,0,0]
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

    def distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

    def setVehicleMode(self, mode):
        self.vehicle.mode = VehicleMode(mode)

    def guidanceLoop(self):

        if self.mission:
            globPose = self.vehiclePose
            task = self.mission.tasks[0]

            if   task.type == Task.TYPE.TAKEOFF:
                if not task.active:
                    task.start()
                if not self.armingInProgress:
                    self.arm_vehicle()
                    task.target = [globPose[0], globPose[1], task.target[2]]

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

                # Check completed
                if self.distance(globPose[:3], task.target) < 0.1: #TARGETREACHPRECISION
                    self.mission.remove(0)

            elif task.type == Task.TYPE.FOLLOW:
                if not task.active:
                    task.start()
                if self.followObjPos == -1
                    # task.target = self.vehiclePose[:3]
                    pass # goes to last observed position then hovers
                else:
                    task.target = self.followObjPos

                # Check completed
                if task.istimeout():
                    self.mission.remove(0)

            # Set actuatorTarget
            self.actuatorTarget = task.target


    # This actuator loop is executed in every cfg['actuatorPeriod'] sec
    def actuatorLoop(self):
        # Step actuator loop
        self.actuator.step(self.vehiclePose, self.actuatorTarget)


    def estimatorLoop(self):
        poses = self.estimator.getPoses()
        self.vehiclePose = poses[0]
        if poses[1]:
            self.followObjPos = poses[1][:3]
        else:
            self.followObjPos = -1


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

    def run(self):

        cfg = self.cfg

        #self.udpsock = socket.socket(socket.AF_INET, # Internet
        #                     socket.SOCK_DGRAM) # UDP
        #self.udpsock.bind((self.cfg['UDP_IP'], self.cfg['UDP_PORT']))
                #self.poscapturethread = threading.Thread(target=self.positionReceiver)

        # Create guidance, estimator, and actuator loops
        self.cthread = periodicrun(cfg['guidancePeriod'],  self.guidanceLoop,  accuracy=0.001)
        self.athread = periodicrun(cfg['actuatorPeriod'],  self.actuatorLoop,  accuracy=0.001)
        self.pthread = periodicrun(cfg['estimatorPeriod'], self.estimatorLoop, accuracy=0.001)

        # Crete actuator and estimator
        self.actuator = Actuator(vehicle, cfg)
        self.estimator = Estimator(cfg['UDP_IP'], cfg['UDP_PORT'], cfg['VICON_DRONENAME'])

        # Start running the estimator and connect to the vehicle
        self.estimator.run()
        self.connectToVehicle()

        # Start machines
        #self.poscapturethread.start()
        self.pthread.run_thread()
        self.cthread.run_thread()
        self.athread.run_thread()

        self.monitor()

    def stop(self):
        logger.debug('Controller shut down.')
        self.athread.interrupt()
        self.cthread.interrupt()
        self.pthread.interrupt()
        self.vehicle.close()


    def join(self):
        #self.poscapturethread.join()
        self.athread.join()
        self.cthread.join()
        self.pthread.join()

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
