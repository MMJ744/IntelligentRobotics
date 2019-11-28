from State import State
from StateMachine import StateMachine

global groupSize = 9999
global groupTable = -1

global tables = {
    1 : {
        places: 6,
        avaliable: False,
        id: 1
    },
    2 : {
        places: 3,
        avaliable: True,
        id: 2
    }
}

class AskBooking(State):
    def run(self):
        speech("Do you have a booking")
        response = listen()
        self.next(response)
    def next(self, input):
        if ('yes','okay') in input:
            return WaitingTask.bd
        if ('no') in input:
            return WaitingTask.ags
        return WaitingTask.ua

class AskGroupSize(State):
    def run(self):
        speech("How many people are their in your group")
        response = listen()
        self.next(response)
    def next(self, input):
        amount = filter(lambda x: x.isdigit(), input)
        if amount = '':
            return WaitingTask.ua
        else:
            groupSize = int(amount)
            return WaitingTask.cg

class GuideToTable(State):
    def run(self):
        speech("Please follow me to your table")
        navigate("table"+groupTable)
        speech("Please take a seat someone will be with you in a few minutes")

class GiveWaitingTime(State):
    def run(self):
        speech("Your wait time is 60 minutes, is this okay")
        response = listen()
        self.next(response)
    def next(self, input):
        if ('yes', 'okay') in input:
            return WaitingTask.g
        else:
            print("Done")

class BookingDetails(State):
    def run(self):

    def next(self, input):

class CheckGroup(State):
    def run(self):
        for table in tables:
            if table.avaliable and table.places >= groupSize:
                table.avaliable = False
                groupTable = table.id
                continue
        self.next('')
    def next(self, input):
        if (groupTable != -1):
            return WaitingTask.g
        else:
            return WaitingTask.wt

class UnknownAnswer(State):
    def run(self):
        speech("Sorry I didn't understand your answer, please can you repeat it")

class WaitingTask(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, WaitingTask.askbooking)

WaitingTask.ab = AskBooking()
WaitingTask.ags = AskGroupSize()
WaitingTask.g = GuideToTable()
WaitingTask.wt = GiveWaitingTime()
WaitingTask.bd = BookingDetails()
WaitingTask.ua = UnknownAnswer()
