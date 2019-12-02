from State import State
from StateMachine import StateMachine
from Speech import navigate #, speech,  listen


class CheckEntry(State):

    def run(self):
        navigate(self.model.locations["FrontDesk"])

    def next(self, instance, input):
        self.reverse = False
        return CheckTables()


class CheckTables(State):

    def run(self):
        tbls = [filter(lambda tbl: not tbl["available"], self.model.tables)]
        tbls.sort(reverse=self.reverse)
        for table in tbls:
            navigate("table" + table["id"])

    def next(self, instance, input):
        if self.reverse:
            return CheckEntry()
        else:
            return CheckTables()


class CheckKitchen(State):

    def run(self):
        navigate(self.model.locations["Kitchen"])

    def next(self, instance, input):
        self.reverse = True
        return CheckTables()


class WanderTask(StateMachine):
    def __init__(self, model):
        super(WanderTask, self).__init__(WanderTask.checkEntry, model)
        self.reverse = False


WanderTask.checkEntry = CheckEntry()
WanderTask.checkTables = CheckTables()
WanderTask.checkKitchen = CheckKitchen()

instance = WanderTask()
instance.runAll()

