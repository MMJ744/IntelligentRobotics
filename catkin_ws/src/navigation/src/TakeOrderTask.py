import StateMachine
import State
from Speech import navigateTo, speech, listen


class NavigateToTable(State):
    def run(self):
        navigateTo("table" + str(instance.table.id))

    def next(self):
        return TakeOrderItem()


class TakeOrderItem(State):
    def run(self):
        if instance.order_items_taken > 0:
            speech("Can I take the first order from this table?")
        else:
            speech("Can I take another order?")

        instance.addInput(listen())

    def next(self, input):
        if input == "":
            return UnknownAnswer()
        else:
            print("Send order \'" + input + "\' for table " + str(instance.table.id) + " to the kitchen")
            instance.order_items_taken += 1
            if instance.order_items_taken >= instance.group_size:
                instance.running = False
            else:
                return TakeOrderItem()


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