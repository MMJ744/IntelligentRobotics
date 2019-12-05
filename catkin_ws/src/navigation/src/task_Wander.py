from State import State
from StateMachine import StateMachine
import navTo
import listenpeople as vision
from Speech import speech

class CheckTables(State):

    def run(self, instance):
        tbls = map(lambda x: x["id"],filter(lambda tbl: not tbl["available"], instance.model.tables))
        tbls.sort(reverse=True)
        for table_int in tbls:
            navTo.navigateTo("table" + str(table_int))    
            if vision.are_people:
                self.model.tables[table_int]["available"] = True
        navTo.navigateTo("frontdesk")
        if vision.are_people:
            taskManager.new_task("NewCust")
            speech("I'll be right with you")
        taskManager.new_task("Wander")
        instance.running = False


class WanderTask(StateMachine):
    def __init__(self, model):
        navTo.main()
        StateMachine.__init__(self, CheckTables(), model) 


# WanderTask.checkEntry = CheckEntry()
# WanderTask.checkTables = CheckTables()
# WanderTask.checkKitchen = CheckKitchen()
#
# instance = WanderTask()
# instance.runAll()


import taskManager
