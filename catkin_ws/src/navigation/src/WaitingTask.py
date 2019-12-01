#!/usr/bin/env python
from State import State
from StateMachine import StateMachine
from Speech import speech, navigate, listen
from navController import main, navigateTo


instance = 0


class AskBooking(State):
    global instance
    def run(self):
        speech("Do you have a booking")
        #response = listen()
        response = 'no'
        instance.addInput(response)

    def next(self, input):
        if 'yes' in input:
            return WaitingTask.bookingDetails
        if 'no' in input:
            return WaitingTask.askGroupSize
        return WaitingTask.unknownAnswer

class AskGroupSize(State):
    global instance
    def run(self):
        speech("How many people are there in your group")
        #response = listen()
        response = 9
        instance.addInput(response)

    def next(self, input):
        if input == '':
            return WaitingTask.unknownAnswer
        else:
            instance.groupSize = int(input)
            return WaitingTask.checkGroup

class GuideToTable(State):
    global instance
    def run(self):
        speech("Please follow me to your table")
        navigateTo("table"+str(instance.groupTable))
        speech("Please take a seat someone will be with you in a few minutes")
        instance.addInput('')
        instance.running = False

class GiveWaitingTime(State):
    global instance
    def run(self):
        speech("Your wait time is 60 minutes, is this okay")
        response = listen()
        instance.addInput(response)

    def next(self, input):
        if 'yes' in input:
            return WaitingTask.guideToTable
        else:
            instance.running = False

class BookingDetails(State):
    global instance
    def run(self):
        instance.addInput('')
        instance.running = False


class CheckGroup(State):
    global instance
    def run(self):
        for table in self.model.tables:
            if table['available'] and table['places'] >= instance.groupSize:
                table['available'] = False
                instance.groupTable = table['id']
                break
        instance.addInput('')

    def next(self, input):
        if (instance.groupTable != -1):
            return WaitingTask.guideToTable
        else:
            return WaitingTask.giveWaitingTime

class UnknownAnswer(State):
    global instance
    def run(self):
        speech("Sorry I didn't understand your answer, please can you repeat that")
        instance.addInput('')
        
    def next(self, inputs):
        return instance.previousState

class WaitingTask(StateMachine):
    def __init__(self, model):
        super(WaitingTask, self).__init__(WaitingTask.askBooking, model)
        self.groupTable = -1
        self.groupSize = 99999


WaitingTask.askBooking = AskBooking()
WaitingTask.askGroupSize = AskGroupSize()
WaitingTask.guideToTable = GuideToTable()
WaitingTask.giveWaitingTime = GiveWaitingTime()
WaitingTask.bookingDetails = BookingDetails()
WaitingTask.unknownAnswer = UnknownAnswer()
WaitingTask.checkGroup = CheckGroup()

instance = WaitingTask()
instance.runAll()

