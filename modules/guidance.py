
from modules.mission import Mission
from modules.mission import Task

class GuidanceBase:
    def getTarget(pose):
        raise NotImplementedError

class MissionGuidance(GuidanceBase):
    mission = None
    followObjPos = -1

    def setMission(mission):
        self.mission = mission

    def getTarget(globPose):

        if self.mission:
            if len(self.mission.tasks) == 0:
                self.mission = None
                return None

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
