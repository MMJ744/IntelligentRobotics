from StateMachine import StateMachine
from State import State
from Speech import speech, listen
import listenpeople as vision
import taskManager
import taskExecuter
import navTo


class NavigateToTable(State):
    def run(self, instance):
        navTo.navigateTo("table" + str(instance.table))

    def next(self, instance, input):
        return CheckReady()


class CheckReady(State):
    def run(self, instance):
        if not vision.are_people():
            speech("I can't see anyone")
            instance.running = False
            return
        speech("Are we ready to order?")
        instance.addInput(listen())

    def next(self, instance, input):
        if "no" in input:
            return ComeBackLater()
        elif "yes" in input:
            return TakeFirstOrderItem()
        else:
            return UnknownAnswer()


class ComeBackLater(State):
    def run(self, instance):
        speech("Okay, I will come back later")
        cusID = instance.model.tables[instance.table-1]['customerID']
        taskManager.new_task("TakeOrder", table_number=instance.table, delay=5, customerID=cusID)
        instance.running = False


class TakeFirstOrderItem(State):
    def run(self, instance):
        speech("What is the first order?")
        instance.addInput(listen())

    def next(self, instance, input):
        instance.model.prepend_message("kitchen", "Send order \'" + input + "\' for table " + str(instance.table) + " to the kitchen")
        return CheckMoreOrders()


class CheckMoreOrders(State):
    def run(self, instance):
        speech("Would anyone else like to order?")
        instance.addInput(listen())

    def next(self, instance, input):
        if 'yes' in input:
            return TakeOrderItem()
        elif 'no' in input:
            return Finished()
        else:
            return UnknownAnswer()


class TakeOrderItem(State):
    def run(self, instance):
        speech("Please tell me the order")

        instance.addInput(listen())

    def next(self, instance, input):
        if input == "":
            return UnknownAnswer()
        else:
            print("to:" + str(instance.model))
            instance.model.prepend_message("kitchen", "Send order \'" + input + "\' for table " + str(instance.table) + " to the kitchen")
            return CheckMoreOrders()


class Finished(State):
    def run(self, instance):
        speech("Thank you. Your food will be with you soon")
        cusID = instance.model.tables[instance.table-1]['customerID']
        taskManager.new_task("Deliver", table_number=instance.table, delay=1, customerID=cusID)
        instance.running = False


class UnknownAnswer(State):
    def run(self, instance):
        speech("Sorry, I didn't understand your answer, let me try again")
        instance.addInput('')

    def next(self, instance, input):
        return instance.previousState


class TakeOrderTask(StateMachine):
    def __init__(self, model, table):
        StateMachine.__init__(self, NavigateToTable(), model)
        self.table = table
        self.group_size = model.tables[table-1]
        self.order_items_taken = 0


# TakeOrderTask.navigateToTable = NavigateToTable()
# TakeOrderTask.checkReady = CheckReady()
# TakeOrderTask.comeBackLater = ComeBackLater()
# TakeOrderTask.takeFirstOrderItem = TakeFirstOrderItem()
# TakeOrderTask.checkMoreOrders = CheckMoreOrders()
# TakeOrderTask.takeOrderItem = TakeOrderItem()
# TakeOrderTask.finished = Finished()
# TakeOrderTask.unknownAnswer = UnknownAnswer()
#
# instance = TakeOrderTask(model=None,table=2)
# instance.run_all()
