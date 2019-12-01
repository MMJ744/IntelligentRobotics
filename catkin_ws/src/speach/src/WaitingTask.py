from State import State
from StateMachine import StateMachine
from Speech import speech, navigate


inputs = []
instance = 0

class AskBooking(State):
    def run(self):
        speech("Do you have a booking")
        response = 'no'
        #response = listen()
        global inputs
        inputs.append(response)

    def next(self, input):
        print(input)
        if 'yes' in input:
            return WaitingTask.bookingDetails
        if 'no' in input:
            return WaitingTask.askGroupSize
        return WaitingTask.unknowAnswer

class AskGroupSize(State):
    def run(self):
        speech("How many people are their in your group")
        response = 4
        #response = listen()
        global inputs
        inputs.append(response)

    def next(self, input):
        global instance
        amount = filter(lambda x: x.isdigit(), str(input))
        if amount == '':
            return WaitingTask.unknowAnswer
        else:
            print('people ' + str(amount))
            instance.groupSize = int(amount)
            return WaitingTask.checkGroup

class GuideToTable(State):
    def run(self):
        global instance
        speech("Please follow me to your table")
        navigate("table"+str(instance.groupTable))
        speech("Please take a seat someone will be with you in a few minutes")
        global inputs
        inputs.append('')
        global instance
        instance.running = False

class GiveWaitingTime(State):
    def run(self):
        speech("Your wait time is 60 minutes, is this okay")
        #response = listen()
        response = 'yes'
        global inputs
        inputs.append(response)

    def next(self, input):
        if 'yes' in input:
            return WaitingTask.guideToTable
        else:
            print("Done")
            global instance
            instance.running = False

class BookingDetails(State):
    def run(self):
        print("done")
        global inputs
        inputs.append('')
        global instance
        instance.running = False


class CheckGroup(State):
    def run(self):
        global instance
        for key in instance.tables:
            table = instance.tables[key]
            if table['avaliable'] and table['places'] >= instance.groupSize:
                print(table)
                table['avaliable'] = False
                instance.groupTable = table['id']
                print(table)
                break
        global inputs
        inputs.append('')
        print(instance.groupTable)

    def next(self, input):
        global instance
        if (instance.groupTable != -1):
            return WaitingTask.guideToTable
        else:
            return WaitingTask.giveWaitingTime

class UnknownAnswer(State):
    def run(self):
        speech("Sorry I didn't understand your answer, please can you repeat it")
        global inputs
        inputs.append('')
        
    def next(self, inputs):
        global instance
        return instance.previousState

class WaitingTask(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, WaitingTask.askBooking)
        self.groupTable = -1
        self.groupSize = 99999
        self.tables = {
            1 : {
                "places": 6,
                "avaliable": False,
                'id': 1
            },
            2 : {
                "places": 4,
                "avaliable": True,
                'id': 2
            }
}

WaitingTask.askBooking = AskBooking()
WaitingTask.askGroupSize = AskGroupSize()
WaitingTask.guideToTable = GuideToTable()
WaitingTask.giveWaitingTime = GiveWaitingTime()
WaitingTask.bookingDetails = BookingDetails()
WaitingTask.unknowAnswer = UnknownAnswer()
WaitingTask.checkGroup = CheckGroup()

instance = WaitingTask()
instance.runAll(inputs)

