#!/usr/bin/env python
from State import State
from StateMachine import StateMachine
from Speech import speech, navigate, listen
from navController import navigateTo
import taskManager


class NavigateToFront(State):

    def run(self, instance):
        navigateTo("front")

    def next(self, instance, input):
        return AskBooking()


class AskBooking(State):

    def run(self, instance):
        speech("Do you have a booking")
        # response = listen()
        response = 'no'
        instance.addInput(response)

    def next(self, instance, input):
        if 'yes' in input:
            return BookingDetails()
        if 'no' in input:
            return AskGroupSize()
        return UnknownAnswer()


class AskGroupSize(State):

    def run(self, instance):
        speech("How many people are there in your group")
        # response = listen()
        response = 9
        instance.addInput(response)

    def next(self, instance, input):
        if input == '':
            return UnknownAnswer()
        else:
            instance.group_size = int(input)
            return CheckGroup


class GuideToTable(State):

    def run(self, instance):
        speech("Please follow me to your table")
        navigateTo("table" + str(instance.group_table))
        speech("Please take a seat. Someone will be with you in a few minutes")
        instance.addInput('')
        taskManager.new_task("TakeOrder", table_number=instance.group_table, delay=5)
        instance.running = False


class GiveWaitingTime(State):

    def run(self, instance):
        speech("Your wait time is 60 minutes, is this okay")
        response = listen()
        instance.addInput(response)

    def next(self, instance, input):
        if 'yes' in input:
            return GuideToTable()
        else:
            return BookingDetails()


class BookingDetails(State):

    def run(self, instance):
        instance.addInput('')
        instance.running = False


class CheckGroup(State):

    def run(self, instance):
        for table in instance.tables:
            if table['available'] and table['places'] >= instance.group_size:
                table['available'] = False
                instance.group_table = table['id']
                break
        instance.addInput('')

    def next(self, instance, input):
        if instance.group_table != -1:
            return GuideToTable()
        else:
            return GiveWaitingTime()


class UnknownAnswer(State):

    def run(self, instance):
        speech("Sorry I didn't understand your answer, please can you repeat that")
        instance.addInput('')

    def run(self, instance, inputs):
        return instance.previousState


class NewCustomerTask(StateMachine):
    def __init__(self, model):
        super(NewCustomerTask, self).__init__(AskBooking(), model)
        self.group_table = -1
        self.group_size = 99999

# NewCustomerTask.askBooking = AskBooking()
# NewCustomerTask.askGroupSize = AskGroupSize()
# NewCustomerTask.guideToTable = GuideToTable()
# NewCustomerTask.giveWaitingTime = GiveWaitingTime()
# NewCustomerTask.bookingDetails = BookingDetails()
# NewCustomerTask.unknownAnswer = UnknownAnswer()
# NewCustomerTask.checkGroup = CheckGroup()
#
# instance = NewCustomerTask()
# instance.runAll()
