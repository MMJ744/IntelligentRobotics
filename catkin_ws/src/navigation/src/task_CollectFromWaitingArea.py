from StateMachine import StateMachine
from State import State
import navTo
from speech import speech, listen
import utils
import taskManager


class NavigateToWaitingArea(State):
    def run(self, instance):
        navTo.navigateTo("waitingarea")

    def next(self, instance):
        return ThankForWaiting()


class ThankForWaiting(State):
    def run(self, instance):
        speech("Hi again, thanks for waiting! Your patience is appreciated.")

    def next(self, instance):
        return AskGroupSize()


class AskGroupSize(State):

    def run(self, instance):
        instance.group_size = 99999
        speech("Please remind me how many people there are in your group")
        response = listen()
        instance.addInput(response)

    def next(self, instance, input):
        print(input)
        if input is None or input == '':
            return UnknownAnswer()
        else:
            answer = utils.getNum(input)
            if answer == '' or answer == ' ':
                return UnknownAnswer()
            instance.group_size = int(answer)
            return CheckGroup()


class CheckGroup(State):

    def run(self, instance):
        speech("Table for " + str(instance.group_size))
        print("group=" + str(instance.group_size))

    def next(self, instance, input):

        big_tables = list(filter(lambda t: t['places'] >= instance.group_size, instance.model.tables))

        if big_tables == []:
            speech("Sorry, we don't have any tables in the restaurant big enough to seat " + str(instance.group_size) + " people.")
            return Evict()

        big_avail_tables = list(filter(lambda t: t['available'], big_tables))

        if big_avail_tables == []:
            speech("Sorry, all our tables of size " + str(instance.group_size) + " or above are still busy.")
            return GiveWaitingTime()

        big_avail_tables.sort(key=(lambda x: x['places']))
        instance.group_table = big_avail_tables[0]['id']
        instance.model.tables[instance.group_table]['customerID'] +=1
        instance.model.tables[instance.group_table]['avaliable'] = False
        return GuideToTable()


class UnknownAnswer(State):

    def run(self, instance):
        speech("Sorry, I didn't understand your answer, let me try again")
        instance.addInput('')

    def next(self, instance, input):
        print(instance.previousState)
        return instance.previousState


class GuideToTable(State):

    def run(self, instance):
        speech("Please follow me")
        navTo.navigateTo("table" + str(instance.group_table))
        speech("Please take a seat. Someone will be with you in a few minutes")
        instance.addInput('')
        cusID = instance.model.tables[instance.table_number-1]['customerID']
        taskManager.new_task("TakeOrder", table_number=instance.group_table, delay=0.5, customerID=cusID)
        instance.model.tables[instance.table_number]
        instance.running = False


class GiveWaitingTime(State):

    def run(self, instance):
        speech("The revised estimate for your wait time is another 5 minutes. Is this okay?")
        response = listen()
        instance.addInput(response)

    def next(self, instance, input):
        result = utils.getYesNo(input)
        if 'yes' == result:
            return Leave()
        else:
            return Goodbye()


class Leave(State):
    def run(self, instance):
        speech("Thanks again for your patience. I'll be back soon.")
        taskManager.new_task("CollectFromWaitingArea", table_number=None, delay=5, customerID=-1)
        instance.running = False


class Evict(State):
    def run(self, instance):
        speech("On behalf and myself and various carbon-based associates, we sincerely apologies for the confusion,"
               "but we will not be able to serve you today. Please leave the premises whenever you're ready.")
        instance.running = False


class Goodbye(State):
    def run(self, instance):
        speech("Sorry about that. Goodbye")
        instance.running = False


class CollectFromWaitingAreaTask(StateMachine):
    def __init__(self, model):
        StateMachine.__init__(self, NavigateToWaitingArea(), model)
