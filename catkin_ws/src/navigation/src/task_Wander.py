from State import State
from StateMachine import StateMachine
import navTo


class CheckTables(State):

    def run(self, instance):
        print('im running')
        tbls = map(lambda x: x["id"],filter(lambda tbl: not tbl["available"], instance.model.tables))
        tbls.sort(reverse=True)
        print(instance.model.tables)
        print(tbls)
        for table_int in tbls:
            print(table_int)
            navTo.navigateTo("table" + str(table_int))    
            # if vision.areHumans:
            # self.model.tables[table]["available"] = True
        print("c")
        navTo.navigateTo("frontdesk")
        print('d')
        
        # if vision.areHuman:
        taskManager.new_task("NewCust")
        taskManager.new_task("Wander")
        instance.running = False


class WanderTask(StateMachine):
    def __init__(self, model):
        print("a")
        print("b")
        StateMachine.__init__(self, CheckTables(), model) 


# WanderTask.checkEntry = CheckEntry()
# WanderTask.checkTables = CheckTables()
# WanderTask.checkKitchen = CheckKitchen()
#
# instance = WanderTask()
# instance.runAll()


import taskManager
