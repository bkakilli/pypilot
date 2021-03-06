import time, math, threading
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from modules.mission import Mission
from modules.mission import Task

class GuidanceBase:
    def getTarget(self, pose):
        raise NotImplementedError

class MissionGuidance(GuidanceBase):
    mission = None
    followObjPos = -1
    cfg = None
    armingThread = None

    def __init__(self, cfg, logger, vehicle):
        self.cfg = cfg
        self.logger = logger
        self.vehicle = vehicle
        self.armingInProgress = False

    def setMission(self, mission):
        self.mission = mission


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

    def getTarget(self, globPose):

        if self.mission:
            if len(self.mission.tasks) == 0:
                self.mission = None
                return None

            task = self.mission.tasks[0]

            if   task.type == Task.TYPE.TAKEOFF:
                if not task.active:
                    task.start()
                if not self.armingInProgress and not self.vehicle.armed:
                    task.target = [globPose[0], globPose[1], task.target[2]]
                    self.armingThread = threading.Thread(target=self.arm_vehicle)
                    self.armingThread.start()

                # Check completed
                if self.distance(globPose[:3], task.target) < 0.1: #TARGETREACHPRECISION
                    self.mission.remove(0)
                    self.armingThread.join()

            elif task.type == Task.TYPE.LAND:
                if not task.active:
                    task.start()
                self.vehicle.mode = VehicleMode('LAND')

                # Check completed
                if self.vehicle.mode == 'LAND':
                    self.mission.remove(0)

            elif task.type == Task.TYPE.HOVER:
                if not task.active:
                    task.target = self.mission.previousTarget[:3]
                    task.start()

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
                if self.followObjPos == -1:
                    # task.target = self.vehiclePose[:3]
                    pass # goes to last observed position then hovers
                else:
                    task.target = self.followObjPos

                # Check completed
                if task.istimeout():
                    self.mission.remove(0)

            return task.target

        return None
