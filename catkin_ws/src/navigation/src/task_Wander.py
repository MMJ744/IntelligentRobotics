from State import State
from StateMachine import StateMachine
from navController import navigateTo


class CheckTables(State):

    def run(self, instance):
        tbls = map(lambda x: x["id"],filter(lambda tbl: not tbl["available"], instance.model.tables))
        # tbls.sort(reverse=True)
        for table_int in tbls:
            print("table" + str(table_int))
            navigateTo("table" + str(table_int))
            # if vision.areHumans:
            #   self.model.tables[table]["available"] = True
        print("frontdesk")
        navigateTo("frontdesk")
        # if vision.areHuman:
        taskManager.new_task("NewCustomer")
        taskManager.new_task("Wander")
        instance.running = False


class WanderTask(StateMachine):
    def __init__(self, model):
        StateMachine.__init__(self, CheckTables(), model)


# WanderTask.checkEntry = CheckEntry()
# WanderTask.checkTables = CheckTables()
# WanderTask.checkKitchen = CheckKitchen()
#
# instance = WanderTask()
# instance.runAll()


import taskManager
