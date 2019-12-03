from datetime import datetime as dt
import rospy


def create(task_msg):
    if task_msg.type == "CheckUp":
        task_info = CheckUp(task_msg.created_at, task_msg.table_number)
    elif task_msg.type == "CollectPayment":
        task_info = CollectPayment(task_msg.created_at, task_msg.table_number)
    elif task_msg.type == "NewCustomer":
        task_info = NewCustomer(task_msg.created_at)
    elif task_msg.type == "TakeOrder":
        task_info = TakeOrder(task_msg.created_at, task_msg.table_number)
    elif task_msg.type == "Wander":
        task_info = Wander(task_msg.created_at)
    else:
        raise NotImplementedError

    return task_info


def new(task_type, table_number, delay):
    created_at = dt.now() + rospy.Duration(delay*60)

    if task_type == "CheckUp":
        task_info = CheckUp(created_at, table_number)
    elif task_type == "CollectPayment":
        task_info = CollectPayment(created_at, table_number)
    elif task_type == "NewCustomer":
        task_info = NewCustomer(created_at)
    elif task_type == "TakeOrder":
        task_info = TakeOrder(created_at, table_number)
    elif task_type == "Wander":
        task_info = Wander(created_at)
    else:
        raise NotImplementedError

    return task_info

def get_priority_level(task_type):
    priority_multipliers = {
        "IMMEDIATE": 4,
        "HIGH": 3,
        "MID": 2,
        "LOW": 1,
        "BASE": 0
    }
    priority_levels = {
        "Wander": "BASE",
        "NewCustomer": "MID",
        "CollectPayment": "MID",
        "CheckUp": "LOW",
        "TakeOrder": "HIGH",
        "Deliver": "IMMEDIATE"
    }
    return priority_multipliers[priority_levels[task_type]]


class Base:
    def __init__(self, task_type, time, table_number=None):
        """
        timestamp and priority level
        """
        self.type = task_type
        self.table_number = table_number

        if time is None:
            self.time_created = dt.now()
        else:
            self.time_created = time
        self.priority_level = get_priority_level(self.type)
        self.update_priority()

    def get_priority(self):
        return (self.time_created - dt.now()) * self.priority_level

    def update_priority(self):  # hot fix - unused
        self.priority = self.get_priority()

    def __lt__(self, other):
        return self.get_priority() < other.get_priority()


class Wander(Base):
    def __init__(self, time=None):
        super().__init__("Wander", time)


class NewCustomer(Base):
    def __init__(self, time=None):
        super().__init__("NewCustomer", time)


class CheckUp(Base):
    def __init__(self, table_number, time=None):
        super().__init__("CheckUp", time, table_number=table_number)


class TakeOrder(Base):
    def __init__(self, table_number, time=None):
        super().__init__("TakeOrder", time, table_number=table_number)


class Deliver(Base):
    def __init__(self, table_number, time=None):
        super().__init__("Deliver", time, table_number=table_number)


class CollectPayment(Base):
    def __init__(self, table_number, time=None):
        super().__init__("CollectPayment", time, table_number=table_number)
