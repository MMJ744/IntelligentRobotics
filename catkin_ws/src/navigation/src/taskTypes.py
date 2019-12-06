#!/usr/bin/env python
import rospy
from navigation.msg import Task

def from_msg(task_msg):
    if task_msg.task_type == "Checkup":
        task_info = Checkup(table_number=task_msg.table_number, time=task_msg.created_at, customerID=task_msg.customerID)
    elif task_msg.task_type == "CollectPayment":
        task_info = CollectPayment(table_number=task_msg.table_number, time=task_msg.created_at, customerID=task_msg.customerID)
    elif task_msg.task_type == "NewCust":
        task_info = NewCustomer(time=task_msg.created_at)
    elif task_msg.task_type == "TakeOrder":
        task_info = TakeOrder(table_number=task_msg.table_number, time=task_msg.created_at, customerID=task_msg.customerID)
    elif task_msg.task_type == "Wander":
        task_info = Wander(time=task_msg.created_at)
    elif task_msg.task_type == "Deliver":
        task_info = Deliver(table_number=task_msg.table_number, time=task_msg.created_at, customerID=task_msg.customerID)
    else:
        raise NotImplementedError

    return task_info


def new(task_type, table_number, delay, customerID):
    created_at = rospy.Time.now() + rospy.Duration(delay*60)

    if task_type == "Checkup":
        task_info = Checkup(time=created_at, table_number=table_number, customerID=customerID)
    elif task_type == "CollectPayment":
        task_info = CollectPayment(time=created_at, table_number=table_number, customerID=customerID)
    elif task_type == "NewCust":
        task_info = NewCustomer(time=created_at)
    elif task_type == "TakeOrder":
        task_info = TakeOrder(time=created_at, table_number=table_number, customerID=customerID)
    elif task_type == "Wander":
        task_info = Wander(time=created_at)
    elif task_type == "Deliver":
        task_info = Deliver(time=created_at, table_number=table_number, customerID=customerID)
    elif task_type == "CollectFromWaitingArea":
        task_info = CollectFromWaitingArea(time=created_at, table_number=table_number)
    else:
        raise NotImplementedError

    return task_info

def get_priority_level(task_type):
    priority_multipliers = {
        "IMMEDIATE": float("inf"),
        "HIGH": 3.0,
        "MID": 2.0,
        "LOW": 1.0,
        "BASE": 0.0
    }
    priority_levels = {
        "Wander": "BASE",
        "NewCust": "HIGH",
        "CollectPayment": "MID",
        "Checkup": "LOW",
        "TakeOrder": "HIGH",
        "Deliver": "IMMEDIATE",
        "CollectFromWaitingArea": "HIGH"
    }
    return priority_multipliers[priority_levels[task_type]]


class Base:
    def __init__(self, task_type, time, table_number=None, customerID=-1):
        """
        timestamp and priority level
        """
        self.priority = None
        self.type = task_type
        self.table_number = table_number
        self.customerID = customerID

        if time is None:
            self.time_created = rospy.Time.now()
        else:
            self.time_created = time
        self.priority_level = get_priority_level(self.type)
        self.update_priority()

    def __str__(self):
        o = self.type + "\t"
        if self.table_number is not None:
            o = o + "t(" + str(self.table_number) + ")"
        else:
            o = o + "\t"
        if self.customerID is not None:
            o = o + "c(" + str(self.customerID) + ")"
        else:
            o = o + "\c"
        o = o + "\t@ " + str(self.time_created) + "\tp[ " + str(self.priority) + " ]"

        return o
    
    def get_customerID(self):
        return self.customerID

    def get_priority(self):
        # print("tt\tget_priority\t" + str(self))
        return (self.time_created - rospy.Time.now()).to_sec() * self.priority_level

    def update_priority(self):  # hot fix - unused
        # print("tt\tupdate_priority\t" + str(self))
        self.priority = self.get_priority()
        # print("tt\tupdate_priority\t" + str(self))

    def to_msg(self, finished=False):
        t = Task()
        t.task_type = self.type
        t.created_at = self.time_created

        if self.table_number is None:
            t.table_number = -1
        else:
            t.table_number = self.table_number

        t.finished = finished
        print('this is my id' + str(self.customerID))
        t.customerID = int(self.customerID)

        return t

    def __lt__(self, other):
        return self.priority < other.priority


class Wander(Base):
    def __init__(self, time=None):
        Base.__init__(self, "Wander", time)


class NewCustomer(Base):
    def __init__(self, time=None):
        Base.__init__(self, "NewCust", time)


class Checkup(Base):
    def __init__(self, table_number, customerID, time=None):
        Base.__init__(self, "Checkup", time, table_number=table_number, customerID=customerID)


class TakeOrder(Base):
    def __init__(self, table_number, customerID, time=None):
        Base.__init__(self, "TakeOrder", time, table_number=table_number, customerID=customerID)


class Deliver(Base):
    def __init__(self, table_number, customerID, time=None):
        Base.__init__(self, "Deliver", time, table_number=table_number, customerID=customerID)


class CollectPayment(Base):
    def __init__(self, table_number, customerID, time=None):
        Base.__init__(self, "CollectPayment", time, table_number=table_number, customerID=customerID)


class CollectFromWaitingArea(Base):
    def __init__(self, table_number, time=None):
        Base.__init__(self, "CollectFromWaitingArea", time, table_number=table_number, customerID=customerID)
