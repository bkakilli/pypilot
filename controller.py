# Drone controller package
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017
#
# ToDo:
# Thread based manual command system

import time, math, argparse, imp, logging
import curses

from dronekit import connect, VehicleMode

from modules.periodicrun import periodicrun
from modules.estimator import ViconTrackerEstimator as Estimator
from modules.actuator import SimpleVelocityController as Actuator
from modules.guidance import MissionGuidance as Guidance
from modules.mission import Mission
from modules.mission import Task

class Controller(object):

    def __init__(self, cfg, logger):

        self.cfg = cfg
        self.logger = logger

        self.vehicle = None
        self.vehiclePose = [0,0,0,0,0,0]

        self.guidance = None
        self.actuator = None
        self.estimator = None
        self.actuatorTarget = [0,0,2]
        self.followObjPos = [0,0,0]

        self.armingInProgress = False


    def connectToVehicle(self):
        try:
            self.logger.info('Connecting to vehicle on: %s' % self.cfg['port'])
            self.vehicle = connect(self.cfg['port'], baud=self.cfg['baud'], wait_ready=self.cfg['wait_ready'])
            self.logger.info('Connected to the vehice on %s', self.cfg['port'])
            return True
        except:
            self.logger.debug('Connection exception!')
            return False

    def arm_vehicle(self):
        """
        Arms vehicle
        """
        self.armingInProgress = True

        self.logger.debug("Check if armed for take-off.")
        if not self.vehicle.armed:

            self.logger.info("Switching to STABILIZE mode before arming.")
            while not self.vehicle.mode == "STABILIZE":
                self.vehicle.mode = VehicleMode("STABILIZE")
                time.sleep(1)
            self.logger.info( "Waiting for arming...")
            while not self.vehicle.armed:
                self.vehicle.armed = True
                time.sleep(1)

        self.logger.info("Switching to GUIDED_NOGPS mode.")
        while not self.vehicle.mode == "GUIDED_NOGPS":
            self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
            time.sleep(1)

        self.armingInProgress = False

    def distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)


    def guidanceLoop(self):
        # Set actuatorTarget
        target = guidance.getTarget()
        if target:
            self.actuatorTarget = target


    # This actuator loop is executed in every cfg['actuatorPeriod'] sec
    def actuatorLoop(self):
        # Step actuator loop
        self.actuator.step(self.vehiclePose, self.actuatorTarget)


    def estimatorLoop(self):
        poses = self.estimator.getPoses()
        self.vehiclePose = poses[0]
        if poses[1]:
            guidance.followObjPos = poses[1][:3]
        else:
            guidance.followObjPos = -1

    # Curses window
    def curses_monitor(self, win):
        win.nodelay(True)
        key=""
        win.clear()
        win.addstr("Detected key:")
        while True:
            try:
                key = win.getkey()
                #win.clear()
                #win.addstr("Detected key:")
                #win.addstr(str(key))
                key = str(key)
                if key == 'KEY_BACKSPACE':
                    break

                elif key == "KEY_F(10)":
                    self.vehicle.mode = VehicleMode('GUIDED_NOGPS')

                elif key == "KEY_F(11)":
                    self.vehicle.mode = VehicleMode('STABILIZE')

                elif key == "KEY_F(12)":
                    mission = Mission()

                    task = Task(Task.TYPE.TAKEOFF)
                    task.target = [0,0,2]
                    mission.appendTask(task)

                    task = Task(Task.TYPE.HOVER)
                    task.duration = 3
                    mission.appendTask(task)

                    task = Task(Task.TYPE.LAND)
                    mission.appendTask(task)

                    self.guider.setMission(mission)

            except Exception as e:
                # No input
                pass

        self.stop()

    def run(self):

        cfg = self.cfg

        # Create guidance, estimator, and actuator loops
        self.gthread = periodicrun(cfg['guidancePeriod'],  self.guidanceLoop,  accuracy=0.001)
        self.athread = periodicrun(cfg['actuatorPeriod'],  self.actuatorLoop,  accuracy=0.001)
        self.pthread = periodicrun(cfg['estimatorPeriod'], self.estimatorLoop, accuracy=0.001)

        # Crete actuator and estimator
        self.guidance = Guidance()
        self.actuator = Actuator(self.vehicle, self.cfg)
        self.estimator = Estimator(cfg['UDP_IP'], cfg['UDP_PORT'], cfg['VICON_DRONENAME'])

        # Start running the estimator and connect to the vehicle
        if not self.connectToVehicle():
            return
        self.estimator.run()
        # Start machines
        self.pthread.run_thread()
        self.gthread.run_thread()
        self.athread.run_thread()

    def stop(self):
        self.logger.debug('Controller shut down.')

        # Stop machines
        self.athread.interrupt()
        self.gthread.interrupt()
        self.pthread.interrupt()

        # Stop estimator and close vehicle
        self.estimator.stop()
        if self.vehicle:
            self.vehicle.close()


    def join(self):
        self.athread.join()
        self.gthread.join()
        self.pthread.join()

#######################  Main Program  ##########################

def test(cfg):
    pass

if __name__ == '__main__':

    # start logger
    logger = logging.getLogger('Controller')
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(name)s: %(message)s')

    ch = logging.StreamHandler()
    fh = logging.FileHandler('test.log')
    ch.setFormatter(formatter)
    fh.setFormatter(formatter)

    logger.addHandler(ch)
    logger.addHandler(fh)


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
        fh.setFormatter(formatter)
        logger.setLevel(logging.DEBUG)
        logger.debug('Verbose level is set 2. Everything will be displayed as much as possible.')
        print 'test'

    logger.info('test')
    #test(cfg)
    controller = Controller(cfg, logger)
    controller.run()

    curses.wrapper(controller.curses_monitor)

    controller.join()
