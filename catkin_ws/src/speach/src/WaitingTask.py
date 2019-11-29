from State import State
from StateMachine import StateMachine
from Speech import listen, speech


class AskBooking(State):
    def run(self):
        speech("Do you have a booking")
        response = listen()
        self.next(response)

    def next(self, input):
        if ('yes','okay') in input:
            return WaitingTask.bookingDetails
        if ('no') in input:
            return WaitingTask.askGroupSize
        return WaitingTask.unknowAnswer

class AskGroupSize(State):
    def run(self):
        speech("How many people are their in your group")
        response = listen()
        self.next(response)

    def next(self, input):
        amount = filter(lambda x: x.isdigit(), input)
        if amount == '':
            return WaitingTask.unknowAnswer
        else:
            WaitingTask().groupSize = int(amount)
            return WaitingTask.checkGroup

class GuideToTable(State):
    def run(self):
        speech("Please follow me to your table")
        navigate("table"+WaitingTask().groupTable)
        speech("Please take a seat someone will be with you in a few minutes")

class GiveWaitingTime(State):
    def run(self):
        speech("Your wait time is 60 minutes, is this okay")
        response = listen()
        self.next(response)

    def next(self, input):
        if ('yes', 'okay') in input:
            return WaitingTask.guideToTable
        else:
            print("Done")

class BookingDetails(State):
    def run(self):
        print("done")


class CheckGroup(State):
    def run(self):
        for table in WaitingTask().tables:
            if table.avaliable and table.places >= WaitingTask().groupSize:
                table.avaliable = False
                WaitingTask().groupTable = table.id
                continue
        self.next('')

    def next(self, input):
        if (WaitingTask().groupTable != -1):
            return WaitingTask.guideToTable
        else:
            return WaitingTask.giveWaitingTime

class UnknownAnswer(State):
    def run(self):
        speech("Sorry I didn't understand your answer, please can you repeat it")
        
    def next(self, inputs):
        return WaitingTask().previousState

class WaitingTask(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, WaitingTask.askBooking)
        self.groupTable = -1
        self.groupSize = 99999
        self.tables = {
            1 : {
                "places": 6,
                "avaliable": False,
                id: 1
            },
            2 : {
                "places": 3,
                "avaliable": True,
                id: 2
            }
}

WaitingTask.askBooking = AskBooking()
WaitingTask.askGroupSize = AskGroupSize()
WaitingTask.guideToTable = GuideToTable()
WaitingTask.giveWaitingTime = GiveWaitingTime()
WaitingTask.bookingDetails = BookingDetails()
WaitingTask.unknowAnswer = UnknownAnswer()
WaitingTask.checkGroup = CheckGroup()

def navigate(where):
    print('Going to ' + where)
