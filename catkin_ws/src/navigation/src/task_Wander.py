from State import State
from StateMachine import StateMachine
import navTo
import listenpeople as vision
from Speech import speech
import rospy

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
            if not vision.are_people():
                speech("It looks like this table is empty. I'll mark it as available")
                instance.model.tables[table_int-1]["available"] = True
        navTo.navigateTo("frontdesk")
        r = rospy.Rate(1)
        r.sleep()
        if vision.are_people():
            taskManager.new_task("NewCust")
            speech("I'll be right with you")
        taskManager.new_task("Wander")
        r.sleep()
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
