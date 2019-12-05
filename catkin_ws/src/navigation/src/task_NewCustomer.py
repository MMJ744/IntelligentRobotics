#!/usr/bin/env python
from State import State
from StateMachine import StateMachine
from Speech import speech, navigate, listen
import navTo
import taskManager
import utils


class NavigateToFront(State):

    def run(self, instance):
        navTo.navigateTo("frontdesk")

    def next(self, instance, input):
        return AskBooking()


class AskBooking(State):

    def run(self, instance):
        speech("Do you have a booking?")
        response = listen()
        instance.addInput(response)

    def next(self, instance, input):
        if 'yes' in input:
            return BookingDetails()
        if 'no' in input:
            return AskGroupSize()
        return UnknownAnswer()


class AskGroupSize(State):

    def run(self, instance):
        instance.group_size = 99999
        speech("How many people are there in your group?")
        response = listen()
        instance.addInput(response)

    def next(self, instance, input):
        print(input)
        if input is None or input == '':
            return UnknownAnswer()
        else:
            answer = utils.convertToNum(input)
            if answer == '' or answer == ' ': 
                return UnknownAnswer()
            instance.group_size = int(answer)
            return CheckGroup()


class GuideToTable(State):

    def run(self, instance):
        speech("Please follow me to your table")
        navTo.navigateTo("table" + str(instance.group_table))
        speech("Please take a seat. Someone will be with you in a few minutes")
        instance.addInput('')
        taskManager.new_task("TakeOrder", table_number=instance.group_table, delay=0.5)
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
        instance.group_table = None
        print("group=" + str(instance.group_size))
        for table in instance.model.tables:
            print("table? " + str(table))
            print("\tavail:" + str(table['available']) + "\tplaces:" + str(table['places'] >= instance.group_size))
            if table['available'] and table['places'] >= instance.group_size:
                print(table['id'])
                table['available'] = False
                instance.group_table = table['id']
                break
        instance.addInput('')

    def next(self, instance, input):
        if instance.group_table is not None:
            return GuideToTable()
        else:
            return GiveWaitingTime()


class UnknownAnswer(State):

    def run(self, instance):
        speech("Sorry, I didn't understand your answer, let me try again")
        instance.addInput('')

    def next(self, instance, input):
        print(instance.previousState)
        return instance.previousState


class NewCustomerTask(StateMachine):
    def __init__(self, model):
        StateMachine.__init__(self, NavigateToFront(), model)
        self.group_table = None
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
