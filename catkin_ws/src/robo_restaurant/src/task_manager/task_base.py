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


class Wander():

    def __init__(self):
        """
        timestamp and priority level
        """
        self.timeCreated = dt.now()
        self.priority = PriorityLevels.BASE

    def run(self):
        """
        Starts the task and contains the logic of the task

        broadcasts task completion on /control
        """
        raise NotImplementedError()

    def interrupt(self):
        """
        Interrupts the current task cleanly so it can resume
        """
        raise NotImplementedError()

class GreetCustomer():

    def __init__(self):
        """
        timestamp and priority level
        """
        self.timeCreated = dt.now()
        self.priority = PriorityLevels.LOW

    def run(self):
        """
        Starts the task and contains the logic of the task

        broadcasts task completion on /control
        """
        raise NotImplementedError()

    def interrupt(self):
        """
        Interrupts the current task cleanly so it can resume
        """
        raise NotImplementedError()
