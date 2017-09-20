# Drone controller package
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017
#
# ToDo:
#

import time, math, argparse, imp, logging
import curses

from dronekit import connect, VehicleMode

from modules.periodicrun import periodicrun

class Controller(object):

    def __init__(self, cfg, logger):

        self.cfg = cfg
        self.logger = logger

        self.vehicle = None
        self.vehiclePose = [0,0,0,0,0,0]

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
        target = guidance.getTarget()
        if target:
            self.actuatorTarget = target


    # This actuator loop is executed in every cfg['actuatorPeriod'] sec
    # It is responsible for sending attitude commands to the vehicle
    def actuatorLoop(self):
        # Step actuator loop
        self.actuator.step(self.vehiclePose, self.actuatorTarget)


    # Estimator loop reads the most updated poses from estimator and writes
    # into the state object
    def estimatorLoop(self):
        poses = self.estimator.getPoses()
        self.vehiclePose = poses[0]
        if poses[1]:
            guidance.followObjPos = poses[1][:3]
        else:
            guidance.followObjPos = -1


    def run(self):

        cfg = self.cfg

        # Create guidance, estimator, and actuator loops
        self.gthread = periodicrun(cfg['guidancePeriod'],  self.guidanceLoop,  accuracy=0.001)
        self.athread = periodicrun(cfg['actuatorPeriod'],  self.actuatorLoop,  accuracy=0.001)
        self.pthread = periodicrun(cfg['estimatorPeriod'], self.estimatorLoop, accuracy=0.001)

        # Load and create schemes
        Estimator = getattr(imp.load_source('estimator', 'modules/estimator.py'), cfg['EstimatorScheme'])
        Actuator  = getattr(imp.load_source('actuator',  'modules/actuator.py'),  cfg['ActuatorScheme'])
        Guidance  = getattr(imp.load_source('guidance',  'modules/guidance.py'),  cfg['GuidanceScheme'])

        self.guidance = Guidance(cfg)
        self.actuator = Actuator(cfg, self.vehicle)
        self.estimator = Estimator(cfg)

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

    def test(self):
        self.logger.debug('Test called.')

#######################  User Interface  ##########################

# Curses window
def curses_monitor(win, controller):
    from modules.mission import Mission
    from modules.mission import Task

    win.nodelay(True)
    key=""
    win.clear()
    while True:
        try:
            key = win.getkey()
            #win.clear()
            #win.addstr("Detected key:")
            #win.addstr(str(key))
            key = str(key)
            if key == 'KEY_BACKSPACE':
                break

            if key == 'r':
                controller.test()

            elif key == "KEY_F(10)":
                controller.vehicle.mode = VehicleMode('GUIDED_NOGPS')

            elif key == "KEY_F(11)":
                controller.vehicle.mode = VehicleMode('STABILIZE')

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

                controller.guidance.setMission(mission)

        except Exception as e:
            # No input
            pass

    controller.stop()

#######################  Main Program  ##########################

def test(cfg):
    pass

if __name__ == '__main__':

    # start logger
    logger = logging.getLogger('Controller')
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(name)s: %(message)s\r')

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
        formatter = logging.Formatter('%(name)s | %(asctime)s: %(message)s\r')
        ch.setFormatter(formatter)
        fh.setFormatter(formatter)
        logger.setLevel(logging.DEBUG)
        logger.debug('Verbose level is set 2. Everything will be displayed as much as possible.')
        print 'test'

    logger.info('test')
    controller = Controller(cfg, logger)
    controller.run()
    curses.wrapper(curses_monitor, controller)
    controller.join()
