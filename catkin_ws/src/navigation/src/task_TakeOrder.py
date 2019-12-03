from StateMachine import StateMachine
from State import State
from Speech import speech, listen
import taskManager
import taskExecuter
from navController import navigateTo


class NavigateToTable(State):
    def run(self, instance):
        navigateTo("table" + str(instance.table.id))

    def next(self, instance, input):
        return CheckReady()


class CheckReady(State):
    def run(self, instance):
        speech("Hello again. Are we ready to order?")
        instance.addInput(listen())

    def next(self, instance, input):
        if input[2:] == "no":
            return ComeBackLater()
        elif input[3:] == "yes":
            return TakeFirstOrderItem()
        else:
            return UnknownAnswer()


class ComeBackLater(State):
    def run(self, instance):
        speech("Okay, I will come back later")
        taskManager.new_task("TakeOrder", table_number=instance.table, delay=2)
        instance.running = False


class TakeFirstOrderItem(State):
    def run(self, instance):
        speech("What is the first order?")
        instance.addInput(listen())

    def next(self, instance, input):
        taskExecuter.send_message("kitchen", "Send order \'" + input + "\' for table " + str(instance.table.id) + " to the kitchen")
        return CheckMoreOrders()


class CheckMoreOrders(State):
    def run(self, instance):
        speech("Would anyone else like to order")
        instance.addInput(listen())

    def next(self, instance, input):
        if input[3:] == "yes":
            return TakeOrderItem()
        elif input[2:] == "no":
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
            taskExecuter.send_message("kitchen", "Send order \'" + input + "\' for table " + str(instance.table.id) + " to the kitchen")
            return CheckMoreOrders()


class Finished(State):
    def run(self, instance):
        speech("Thank you. Your food will be with you soon")
        instance.running = False


class UnknownAnswer(State):
    def run(self, instance):
        speech("Sorry I didn't understand your answer, please can you repeat that")
        instance.addInput('')

    def next(self, instance, input):
        return instance.previousState


class TakeOrderTask(StateMachine):
    def __init__(self, model, table):
        StateMachine.__init__(self, NavigateToTable(), model)
        self.table = table
        self.group_size = model.tables[table]
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
# instance = TakeOrderTask()
# instance.run_all()
