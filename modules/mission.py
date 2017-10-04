# Defines and creates mission object

from enum import Enum
import time

"""
Tasks:
0. Take-off
1. Take-offTo
2. Land
3. Hover
4. MoveTo
5. LandOn
6. Search-X

A mission will look like:
[Task.TAKEOFF, Task.HOVER, Task.MOVETO, ]
"""

class Task:
    class TYPE(Enum):
        TAKEOFF = 1
        LAND = 2
        HOVER = 3
        MOVETO = 4
        LANDON = 5
        SEARCHX = 5

    active = False
    target = [0,0,0]
    relative = True
    duration = 0
    starttime = 0

    def __init__(self, t):
        self.type = t

    def start(self, duration=0):
        self.active = True
        self.starttime = time.time()

    def istimeout(self):
        if self.duration != 0:
            if time.time()-self.starttime >= self.duration:
                return True
        return False

class Mission:
    tasks = []
    previousTarget = [0,0,0]

    def appendTask(self, t):
        self.tasks.append(t)

    def remove(self, i):
        self.previousTarget = self.tasks[0].target
        self.tasks.pop(0)
