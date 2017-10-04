# Pilot package
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017
#
# ToDo:
# Pass logger to all modules. Maybe a common shared object.

import time, math, imp, logging
from dronekit import connect, VehicleMode

from modules.periodicrun import periodicrun

class Pilot(object):

    def __init__(self, cfg, logger):

        self.cfg = cfg
        self.logger = logger

        self.vehicle = None
        self.vehiclePose = [0,0,0,0,0,0]
        self.homePose = [0,0,0,0,0,0]
        self.setHomeCmd = False

        self.athread = None
        self.gthread = None

        self.guidance = None
        self.actuator = None
        self.estimator = None
        self.actuatorTarget = [0,0,0]
        self.followObjPos = [0,0,0]


    def connectToVehicle(self):
        try:
            self.logger.info('Connecting to vehicle on: %s' % self.cfg['port'])
            self.vehicle = connect(self.cfg['port'], baud=self.cfg['baud'], wait_ready=self.cfg['wait_ready'])
            self.logger.info('Connected to the vehice on %s', self.cfg['port'])
            return True
        except:
            self.logger.debug('Connection exception!')
            return False


    # Guidance loop is responsible for generating higher level targets
    # which most of the time comes from a seperate application, possibly
    # running on some other hardware.
    def guidanceLoop(self):
        # Set actuatorTarget
        target = self.guidance.getTarget(self.vehiclePose)
        if target:
            self.actuatorTarget = target

        #print self.vehiclePose

    # This actuator loop is executed in every cfg['actuatorPeriod'] sec
    # It is responsible for sending attitude commands to the vehicle
    def actuatorLoop(self):
        # Step actuator loop
        self.actuator.step(self.vehiclePose, self.actuatorTarget)

    # Estimator loop reads the most updated poses from estimator and writes
    # into the state object
    def estimatorLoop(self):
        poses = self.estimator.getPoses()
        if len(poses):
            self.vehiclePose = poses[0]
        #print self.vehiclePose
        #if not len(poses)==1:
        #    self.guidance.followObjPos = poses[1][:3]
        #else:
        #    self.guidance.followObjPos = -1

    def updateEstimations(self, poses):
        if self.setHomeCmd:
            for i in range(6):
                self.homePose[i] = poses[0][i]
            self.setHomeCmd = False

        for i in [0,1,2]:
            self.vehiclePose[i] = poses[0][i] - self.homePose[i]
        for i in [3,4,5]:
            self.vehiclePose[i] = poses[0][i] - self.homePose[i]
            if self.vehiclePose[i] > math.pi:
                self.vehiclePose[i] = -2*math.pi + self.vehiclePose[i]
            elif self.vehiclePose[i] < -math.pi:
                self.vehiclePose[i] =  2*math.pi + self.vehiclePose[i]


    def setHome(self):
        self.setHomeCmd = True

    def run(self):

        cfg = self.cfg

        # Connect to the vehicle
        if not self.connectToVehicle():
            self.logger.error('Could not connect to vehicle.')
            return False

        # Load and create schemes
        Estimator = getattr(imp.load_source('estimator', 'modules/estimator.py'), cfg['EstimatorScheme'])
        Actuator  = getattr(imp.load_source('actuator',  'modules/actuator.py'),  cfg['ActuatorScheme'])
        Guidance  = getattr(imp.load_source('guidance',  'modules/guidance.py'),  cfg['GuidanceScheme'])

        # Create machines
        self.logger.debug('Machines are starting.')
        self.guidance = Guidance(cfg, self.logger, self.vehicle)
        self.actuator = Actuator(cfg, self.logger, self.vehicle)
        self.estimator = Estimator(cfg, self.logger, self.updateEstimations)

        # Start position estimator
        if not self.estimator.run():
            self.logger.error('Estimator could not run.')
            return False

        # Create guidance, estimator, and actuator loops then run
        self.gthread = periodicrun(cfg['guidancePeriod'],  self.guidanceLoop,  accuracy=0.001)
        self.athread = periodicrun(cfg['actuatorPeriod'],  self.actuatorLoop,  accuracy=0.001)

        self.gthread.run_thread()
        self.athread.run_thread()

        return True

    def stop(self):
        # Stop machines
        self.athread.interrupt()
        self.gthread.interrupt()

        # Stop estimator and close vehicle
        self.estimator.stop()
        self.vehicle.close()

        self.logger.debug('Pilot shut down.')


    def join(self):
        if self.athread:
            self.athread.join()
        if self.gthread:
            self.gthread.join()

    def test(self):
        self.logger.debug('Test called.')
