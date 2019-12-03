from State import State
from StateMachine import StateMachine
from Speech import navigate #, speech,  listen

import taskManager


class CheckTables(State):

    def run(self, instance):
        tbls = [filter(lambda tbl: not tbl["available"], self.model.tables)]
        tbls.sort(reverse=True)
        for table in tbls:
            navigate("table" + table["id"])
        navigate(self.model.locations["FrontDesk"])
        taskManager.new_task("Wander")
        instance.running = False


class WanderTask(StateMachine):
    def __init__(self, model):
        super(WanderTask, self).__init__(CheckTables(), model)


# WanderTask.checkEntry = CheckEntry()
# WanderTask.checkTables = CheckTables()
# WanderTask.checkKitchen = CheckKitchen()
#
# instance = WanderTask()
# instance.runAll()

