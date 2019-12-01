import datetime.datetime as dt


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
        "Waiting": "MID",
        "CheckUp": "LOW",
        "Order": "HIGH",
        "Deliver": "IMMEDIATE",
    }
    return priority_multipliers[priority_levels[task_type]]


class Base:
    def __init__(self, task_type, time, table_number=None):
        """
        timestamp and priority level
        """
        if not table_number is None:
            self.table_number = table_number
        if time is None:
            self.time_created = dt.now()
        else:
            self.time_created = time
        self.priority_level = get_priority_level(task_type)
        self.update_priority()

    def get_priority(self):
        return (self.time_created - dt.now()) * self.priority_level

    def update_priority(self):
        self.priority = self.get_priority()

class Wander(Base):
    def __init__(self, time=None):
        super().__init__("Wander", time)


class Waiting(Base):
    def __init__(self, time=None):
        super().__init__("Waiting", time)


class CheckUp(Base):
    def __init__(self, table_number, time=None):
        super().__init__("CheckUp", time, table_number=table_number)


class Order(Base):
    def __init__(self, table_number, time=None):
        super().__init__("Order", time, table_number=table_number)


class Deliver(Base):
    def __init__(self, table_number, time=None):
        super().__init__("Deliver", time, table_number=table_number)
