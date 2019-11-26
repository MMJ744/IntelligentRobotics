import datetime.datetime as dt
import threading
from enum import Enum

Priorities = ["BaseTask"]


class Priorities(Enum):
    IMMEDIATE = 0
    HIGH = 1
    MID = 2
    LOW = 3
    BASE = 4


class Task(threading.Thread):

    def __init__(self):
        """
        Superconstructor, handles timestamp and TODO
        """
        timeDateCreated = dt.now()
        self.priority = Priorities
        self.initPriority()

    def initPriority(self):
        """
        Should set priority for task based on initial time, priority level and other parameters as expanded
        """
        raise NotImplementedError()

    def start(self):
        """
        Starts the task running either from fresh or resuming

        end with super(Task, self).start()
        """
        super(Task, self).start()
        raise NotImplementedError()

    def interrupt(self):
        """
        Interrupts the current task cleanly so it can resume
        """
        raise NotImplementedError()
