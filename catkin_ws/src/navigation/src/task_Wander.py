from State import State
from StateMachine import StateMachine
from Speech import navigate #, speech,  listen


class CheckEntry(State):
    global instance

    def run(self):
        navigate(self.model.locations["FrontDesk"])

    def next(self):
        self.reverse = False
        return WanderTask.checkTables


class CheckTables(State):
    global instance

    def run(self):
        tbls = [filter(lambda tbl: not tbl["available"], self.model.tables)]
        tbls.sort(reverse=self.reverse)
        for table in tbls:
            navigate("table" + table["id"])

    def next(self):
        if self.reverse:
            return WanderTask.checkEntry
        else:
            return WanderTask.checkTables


class CheckKitchen(State):
    global instance

    def run(self):
        navigate(self.model.locations["Kitchen"])

    def next(self):
        self.reverse = True
        return WanderTask.checkTables


class WanderTask(StateMachine):
    def __init__(self, model):
        super(WanderTask, self).__init__(WanderTask.checkEntry, model)
        self.reverse = False


WanderTask.checkEntry = CheckEntry()
WanderTask.checkTables = CheckTables()
WanderTask.checkKitchen = CheckKitchen()

instance = WanderTask()
instance.runAll()

