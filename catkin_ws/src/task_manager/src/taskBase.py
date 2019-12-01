import datetime.datetime as dt
from enum import Enum


class PriorityLevels(Enum):
    IMMEDIATE = 0
    HIGH = 1
    MID = 2
    LOW = 3
    BASE = 4


class Wander(time=None):

    def __init__(self, time):
        """
        timestamp and priority level
        """
        self.type = "Wander"
        if time is None:
            self.timeCreated = dt.now()
        else:
            self.timeCreated = time
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


class GreetCustomer(time=None):

    def __init__(self,time):
        """
        timestamp and priority level
        """
        self.type = "GreetCustomer"
        if time is None:
            self.timeCreated = dt.now()
        else:
            self.timeCreated = time
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
