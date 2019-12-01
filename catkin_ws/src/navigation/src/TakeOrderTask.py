import StateMachine
import State
from Speech import navigateTo, speech, listen


class NavigateToTable(State):
    def run(self):
        navigateTo("table" + str(instance.table.id))

    def next(self):
        return CheckReady()


class CheckReady(State):
    def run(self):
        speech("Hello again. Are we ready to order?")
        instance.addInput(listen())

    def next(self):
        if input[2:] == "no":
            return ComeBackLater()
        elif input[3:] == "yes":
            return TakeFirstOrderItem()
        else:
            return UnknownAnswer()


class ComeBackLater(State):
    def run(self):
        speech("Okay, I will come back later")

    def next(self):
        instance.running = False


class TakeFirstOrderItem(State):
    def run(self):
        speech("What is the first order?")
        instance.addInput(listen())

    def next(self, input):
        print("Send order \'" + input + "\' for table " + str(instance.table.id) + " to the kitchen")
        return CheckMoreOrders()


class CheckMoreOrders(State):
    def run(self):
        speech("Would anyone else like to order")
        instance.addInput(listen())

    def next(self, input):
        if input[3:] == "yes":
            return TakeOrderItem()
        elif input[2:] == "no":
            return Finished()
        else:
            return UnknownAnswer()


class TakeOrderItem(State):
    def run(self):
        speech("Please tell me the order")

        instance.addInput(listen())

    def next(self, input):
        if input == "":
            return UnknownAnswer()
        else:
            print("Send order \'" + input + "\' for table " + str(instance.table.id) + " to the kitchen")
            return CheckMoreOrders()


class Finished(State):
    def run(self):
        speech("Thank you. Your food will be with you soon")

    def next(self):
        instance.running = False


class UnknownAnswer(State):
    def run(self):
        speech("Sorry I didn't understand your answer, please can you repeat that")
        instance.addInput('')

    def next(self, inputs):
        return instance.previousState


class TakeOrderTask(StateMachine):
    def __init__(self, table, group_size):
        super(TakeOrderTask, self).__init__(NavigateToTable())
        self.table = table
        self.group_size = group_size
        self.order_items_taken = 0


instance = TakeOrderTask()