#!/usr/bin/env python
from StateMachine import StateMachine
from State import State
from Speech import speech, listen
import taskManager
import navTo
import rospy

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
        if input is None or input == '':
            return instance.currentState
        elif 'go' in input:
            print("Delivering to table " + str(instance.table_number))
            return NavigateToTable()
        else:
            speech("I'll wait here until you tell me to go.")
            return instance.currentState


class NavigateToTable(State):

    def run(self, instance):
        navTo.navigateTo("table" + str(instance.table_number))
        speech("Here is your " + str(instance.food_order))
        listen()
        speech("Please tell me to go when you've got your food")

    def next(self, instance, input):
        return TableWait()


class TableWait(State):
    def run(self, instance):
        response = listen()
        if response is not None and 'go' in response:
            cusID = instance.model.tables[instance.table_number-1]['customerID']
            taskManager.new_task("Checkup", table_number=instance.table_number, delay=1, customerID=cusID)
            r = rospy.Rate(1)
            r.sleep()
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
