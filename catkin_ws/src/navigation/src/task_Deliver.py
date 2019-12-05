#!/usr/bin/env python
from StateMachine import StateMachine
from State import State
from Speech import speech, listen
import taskManager
import navTo


class NavigateToKitchen(State):

    def run(self, instance):
        navTo.navigateTo("kitchen")

    def next(self, instance, input):
        return AskOrder()


class AskOrder(State):

    def run(self, instance):
        speech("What food am I taking to table " + str(instance.table_number))
        instance.food_order = listen()
        instance.addInput(instance.food_order)

    def next(self, instance, input):
        if input == '':
            return UnknownAnswer()
        else:
            speech("Awesome, I'll wait here until you tell me to go.")
            return KitchenWait()


class KitchenWait(State):

    def run(self, instance):
        instance.addInput(listen())

    def next(self, instance, input):
        if 'go' in input:
            return NavigateToTable()
        elif input == '':
            return instance.currentState
        else:
            speech("I'll wait here until you tell me to go.")
            return instance.currentState


class NavigateToTable(State):

    def run(self, instance):
        navTo.navigateTo("table" + str(instance.table_number))
        speech("Here is your " + str(instance.food_order))
        listen()
        speech("I'll wait here until you tell me to go.")

    def next(self, instance, input):
        return TableWait()


class TableWait(State):
    def run(self, instance):
        response = listen()
        if 'go' in response:
            instance.running = False
        instance.addInput(response)

    def next(self, instance, input):
        if input == '':
            return instance.currentState
        else:
            speech("I'll wait here until you tell me to go.")
            return instance.currentState


class UnknownAnswer(State):

    def run(self, instance):
        speech("Sorry, I didn't understand your answer, let me try again")
        instance.addInput('')

    def next(self, instance, input):
        return instance.previousState


class DeliverTask(StateMachine):
    def __init__(self, model, table):
        StateMachine.__init__(self, NavigateToKitchen(), model)
        self.table_number = table

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
