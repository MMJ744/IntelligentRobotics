from State import State
from StateMachine import StateMachine
from Speech import speech, navigate, listen


instance = 0

class AskBooking(State):
    global instance
    def run(self):
        speech("Do you have a booking")
        response = listen()
        instance.addInput(response)

    def next(self, input):
        if 'yes' in input:
            return WaitingTask.bookingDetails
        if 'no' in input:
            return WaitingTask.askGroupSize
        return WaitingTask.unknowAnswer

class AskGroupSize(State):
    global instance
    def run(self):
        speech("How many people are their in your group")
        response = listen()
        instance.addInput(response)

    def next(self, input):
        amount = filter(lambda x: x.isdigit(), str(input))
        if amount == '':
            return WaitingTask.unknowAnswer
        else:
            instance.groupSize = int(amount)
            return WaitingTask.checkGroup

class GuideToTable(State):
    global instance
    def run(self):
        speech("Please follow me to your table")
        navigate("table"+str(instance.groupTable))
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
        for key in instance.tables:
            table = instance.tables[key]
            if table['avaliable'] and table['places'] >= instance.groupSize:
                table['avaliable'] = False
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
    def __init__(self):
        super.__init__(self, WaitingTask.askBooking)
        self.groupTable = -1
        self.groupSize = 99999

WaitingTask.askBooking = AskBooking()
WaitingTask.askGroupSize = AskGroupSize()
WaitingTask.guideToTable = GuideToTable()
WaitingTask.giveWaitingTime = GiveWaitingTime()
WaitingTask.bookingDetails = BookingDetails()
WaitingTask.unknowAnswer = UnknownAnswer()
WaitingTask.checkGroup = CheckGroup()

instance = WaitingTask()
instance.runAll()

