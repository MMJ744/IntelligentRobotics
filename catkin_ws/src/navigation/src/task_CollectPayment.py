from StateMachine import StateMachine
from State import State
from Speech import navigate, speech, listen
import taskManager


class NavigateToTable(State):

    def run(self, instance):
        navigate("table" + str(instance.table_number))

    def next(self, instance, input):
        return AskIfFinished()


class AskIfFinished(State):

    def run(self, instance):
        speech("Has everyone here finished their meal?")
        instance.addInput(listen())

    def next(self, instance, input):
        if input[:2] == "no":
            return Postpone()
        elif input[:3] == "yes":
            return TakePayment()
        else:
            return UnknownAnswer()


class Postpone(State):

    def run(self, instance):
        speech("My apologies, I shall return later")
        taskManager.new_task("CollectPayment", table_number=instance.table_number, delay=7.5)
        instance.running = False


class TakePayment(State):

    def run(self, instance):
        speech("Please present your card to make your payment")
        instance.payment_taken = True  # Le bodge

    def next(self, instance, input):
        if instance.payment_taken:
            return DismissCustomers()
        else:
            return TakePaymentRetry()


class TakePaymentRetry(State):

    def run(self, instance):
        speech("I'm sorry, that payment wasn't successful. Please try again.")

    def next(self, instance, input):
        if instance.payment_taken:
            return DismissCustomers()
        else:
            return DispatchHuman()


class DispatchHuman(State):

    def run(self, instance):
        speech("I'm sorry, I haven't been able to take a payment from you. Please await further assistance")
        print("Table " + str(instance.table.id) + " unprofitable. Please assist")
        instance.running = False


class DismissCustomers(State):

    def run(self, instance):
        speech("Thank you, your payment has been processed. You may now leave.")
        instance.running = False


class UnknownAnswer(State):

    def run(self, instance):
        speech("Sorry I didn't understand your answer, please can you repeat that")
        instance.addInput('')

    def next(self, instance, input):
        return instance.previousState


class CollectPaymentTask(StateMachine):
    def __init__(self, model, table):
        super(CollectPaymentTask, self).__init__(NavigateToTable(), model)
        self.table_number = table
        self.payment_taken = False

# CollectPaymentTask.navigateToTable = NavigateToTable()
# CollectPaymentTask.askIfFinished = AskIfFinished()
# CollectPaymentTask.postpone = Postpone()
# CollectPaymentTask.takePayment = TakePayment()
# CollectPaymentTask.takePaymentRetry = TakePaymentRetry()
# CollectPaymentTask.dispatchHuman = DispatchHuman()
# CollectPaymentTask.dismissCustomers = DismissCustomers()
# CollectPaymentTask.unknownAnswer = UnknownAnswer()
#
# instance = CollectPaymentTask()
# instance.runAll()
