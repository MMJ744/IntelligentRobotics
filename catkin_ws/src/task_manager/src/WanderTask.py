from State import State
from StateMachine import StateMachine
from Speech import speech, navTo, listen


instance = 0


class checkEntry(State):
    global instance
    def run(self):
        navTo(self.door)

    def next(self):
        self.reverse = False
        return WanderTask.checkTables


class checkTables(State):
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


class checkKitchen(State):
    global instance
    def run(self):
        navTo(self.kitchen)

    def next(self):
        self.reverse = True
        return WanderTask.checkTables


class WanderTask(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, WanderTask.checkEntry)
        self.groupTable = -1
        self.groupSize = 99999
        self.tables = [
            {
                "places": 6,
                "avaliable": False,
                'id': 1
            },
            {
                "places": 4,
                "avaliable": True,
                'id': 2
            }
        ]
        self.reverse = False

WanderTask.checkEntry = CheckEntry()
WanderTask.checkTables = CheckTables()
WanderTask.checkKitchen = CheckKitchen()

instance = WanderTask()
instance.runAll()

