import datetime.datetime as dt
import threading
from enum import Enum

Priorities = ["BaseTask"]


class PriorityLevels(Enum):
    IMMEDIATE = 0
    HIGH = 1
    MID = 2
    LOW = 3
    BASE = 4


class TaskBase(threading.Thread):

    def __init__(self):
        """
        Superconstructor, handles timestamp and TODO
        """
        super().__init__()
        self.timeCreated = dt.now()
        self.priority = PriorityLevels
        self.init_priority_level()

    def init_priority_level(self):
        """
        Should set priority for task based on initial time, priority level and other parameters as expanded
        """
        raise NotImplementedError()

    def start(self):
        """
        Starts the task running either from fresh or resuming

        end with super(Task, self).start()
        """
        super().start()
        raise NotImplementedError()

    def interrupt(self):
        """
        Interrupts the current task cleanly so it can resume
        """
        raise NotImplementedError()
