# PyPilot run module
# Author: Burak Kakillioglu
# bkakilli.github.io
# 08/23/2017
#
# ToDo:
# Curses bug fix.

import argparse, logging, sys, imp
import curses

from modules.pilot import Pilot

#######################  User Interface  ##########################

class StreamToLogger(object):
    """
    Fake file-like stream object that redirects writes to a logger instance.
    """
    def __init__(self, logger, log_level=logging.WARNING):
        self.logger = logger
        self.log_level = log_level

    def write(self, buf):
        for line in buf.rstrip().splitlines():
            self.logger.log(self.log_level, line.rstrip())

    def flush(self):
    # create a flush method so things can be flushed when
    # the system wants to. Not sure if simply 'printing'
    # sys.stderr is the correct way to do it, but it seemed
    # to work properly for me.
        pass

try:
    unicode
    _unicode = True
except NameError:
    _unicode = False

class CursesHandler(logging.Handler):
    def __init__(self, screen):
        logging.Handler.__init__(self)
        self.screen = screen
    def emit(self, record):
        try:
            msg = self.format(record)
            screen = self.screen
            fs = "\n%s"
            if not _unicode: #if no unicode support...
                screen.addstr(fs % msg)
                screen.refresh()
            else:
                try:
                    if (isinstance(msg, unicode) ):
                        ufs = u'\n%s'
                        try:
                            screen.addstr(ufs % msg)
                            screen.refresh()
                        except UnicodeEncodeError:
                            screen.addstr((ufs % msg).encode(code))
                            screen.refresh()
                    else:
                        screen.addstr(fs % msg)
                        screen.refresh()
                except UnicodeError:
                    screen.addstr(fs % msg.encode("UTF-8"))
                    screen.refresh()
        except:
            raise

# Curses window
def curses_monitor(win, pilot, logger):
    from modules.mission import Mission
    from modules.mission import Task


    ch = CursesHandler(win)
    formatter = logging.Formatter('%(name)s | %(asctime)s: %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)

    stderr_logger = StreamToLogger(logger)
    sys.stderr = stderr_logger
    
    win.nodelay(True)

    if pilot.run():
    
        while True:
            try:
                key = win.getkey()
                #win.clear()
                #win.addstr("Detected key:")
                #win.addstr(str(key))
                key = str(key)
                if key == 'KEY_BACKSPACE' or key == 'KEY_F(8)':
                    break
                    
                if key == 'r':
                    pilot.test()

                elif key == "KEY_F(10)":
                    pilot.vehicle.mode = VehicleMode('GUIDED_NOGPS')

                elif key == "KEY_F(11)":
                    pilot.vehicle.mode = VehicleMode('STABILIZE')

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

                    pilot.guidance.setMission(mission)

            except Exception as e:
                # No input
                logger.error('Error in curses loop:')
                logger.error(repr(e))
                pass

        pilot.stop()

#######################  Main Program  ##########################


def test(cfg):
    pass

if __name__ == '__main__':

    # arg parse here
    config_path = 'config.py'

    config_module = imp.load_source('config', config_path)
    cfg = config_module.cfg

    # Logging adjustment
    formatter = logging.Formatter('%(name)s: %(message)s\r')
    if cfg['verbose'] == 1:
        logLevel = logging.INFO
    elif cfg['verbose'] == 2:
        formatter = logging.Formatter('%(name)s | %(asctime)s: %(message)s')
        logLevel = logging.DEBUG

    # start logger
    logger = logging.getLogger('PyPilot')
    logger.setLevel(logLevel)

    if True:
        fh = logging.FileHandler('test.log')
        fh.setFormatter(formatter)
        logger.addHandler(fh)

    pilot = Pilot(cfg, logger)
    # Start UI
    if True:
        curses.wrapper(curses_monitor, pilot, logger)
    else:
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)
        logger.addHandler(ch)
        if pilot.run():
            while True:
                choice = raw_input("Make your choice: ")
                if str(choice) == "q":
                    break
                
            pilot.stop()
    pilot.join()
