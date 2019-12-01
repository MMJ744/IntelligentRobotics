from State import State
from StateMachine import StateMachine
from Speech import speech, navTo, listen


instance = 0


class CheckEntry(State):
    global instance

    def run(self):
        navTo(self.door)

    def next(self):
        self.reverse = False
        return WanderTask.checkTables


class CheckTables(State):
    global instance

    def run(self):
        tbls = [filter(lambda x: not x["available"], self.tables)]
        tbls.sort(reverse=self.reverse)
        for table in tbls:
            navTo("table " + table["id"])

    def next(self):
        if self.reverse:
            return WanderTask.checkEntry
        else:
            return WanderTask.checkTables


class CheckKitchen(State):
    global instance

    def run(self):
        navTo(self.kitchen)

    def next(self):
        self.reverse = True
        return WanderTask.checkTables


class WanderTask(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, WanderTask.checkEntry)
        self.tables = [
            {
                "places": 6,
                "available": False,
                'id': 1
            },
            {
                "places": 4,
                "available": True,
                'id': 2
            }
        ]
        self.reverse = False


WanderTask.checkEntry = CheckEntry()
WanderTask.checkTables = CheckTables()
WanderTask.checkKitchen = CheckKitchen()

instance = WanderTask()
instance.runAll()

