from StateMachine import StateMachine
import State
from Speech import navigateTo, speech, listen


class NavigateToTable(State):
    def run(self):
        navigateTo("table" + str(instance.table.id))

    def next(self):
        return AskIfFinished()


class AskIfFinished(State):
    def run(self):
        speech("Has everyone here finished their meal?")
        instance.addInput(listen())

    def next(self, input):
        if input[:2] == "no":
            return Postpone()
        elif input[:3] == "yes":
            return TakePayment()
        else:
            return UnknownAnswer()


class Postpone(State):
    def run(self):
        speech("My apologies, I shall return later")

    def next(self):
        instance.running = False


class TakePayment(State):
    def run(self):
        speech("Please present your card to make your payment")
        instance.payment_taken = True  # Le bodge

    def next(self):
        if instance.payment_taken:
            return DismissCustomers()


class TakePaymentRetry(State):
    def run(self):
        speech("I'm sorry, that payment wasn't successful. Please try again.")

    def next(self):
        if instance.payment_taken:
            return DismissCustomers()
        else:
            return DispatchHuman()


class DispatchHuman(State):
    def run(self):
        speech("I'm sorry, I haven't been able to take a payment from you. Please await further assistance")
        print("Table " + str(instance.table.id) + " unprofitable. Please assist")

    def next(self):
        instance.running = False


class DismissCustomers(State):
    def run(self):
        speech("Thank you, your payment has been processed. You may now leave.")

    def next(self):
        instance.running = False


class UnknownAnswer(State):
    def run(self):
        speech("Sorry I didn't understand your answer, please can you repeat that")
        instance.addInput('')

    def next(self, inputs):
        return instance.previousState


class CollectPaymentTask(StateMachine):
    def __init__(self, model, table):
        super(CollectPaymentTask, self).__init__(CollectPaymentTask.navigateToTable, model)
        self.table = table
        self.payment_taken = False


CollectPaymentTask.navigateToTable = NavigateToTable()
CollectPaymentTask.askIfFinished = AskIfFinished()
CollectPaymentTask.postpone = Postpone()
CollectPaymentTask.takePayment = TakePayment()
CollectPaymentTask.takePaymentRetry = TakePaymentRetry()
CollectPaymentTask.dispatchHuman = DispatchHuman()
CollectPaymentTask.dismissCustomers = DismissCustomers()
CollectPaymentTask.unknownAnswer = UnknownAnswer()

instance = CollectPaymentTask()
instance.runAll()
