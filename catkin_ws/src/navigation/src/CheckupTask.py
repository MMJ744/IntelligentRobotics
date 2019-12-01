from StateMachine import StateMachine
import State
from Speech import navigateTo, speech, listen


class NavigateToTable(State):
    def run(self):
        navigateTo("table" + str(instance.table.id))

    def next(self):
        return PerformCheckup()


class PerformCheckup(State):
    def run(self):
        speech("Is everything okay with your food?")
        instance.addInput(listen())

    def next(self, input):
        if input == "":
            return UnknownAnswer()
        elif input[:2] == "no":
            instance.running = False
        else:
            print("Log checkup comment: \'" + input + "\' on table " + str(instance.table.id))
            return ConfirmComments()


class ConfirmComments(State):
    def run(self):
        speech("Okay, thank you for your comments. They have been submitted to my organic comrades for review")

    def next(self, input):
        instance.running = False


class UnknownAnswer(State):
    def run(self):
        speech("Sorry I didn't understand your answer, please can you repeat that")
        instance.addInput('')

    def next(self, inputs):
        return instance.previousState


class CheckupTask(StateMachine):
    def __init__(self, table):
        super(CheckupTask, self).__init__(NavigateToTable())
        self.table = table


instance = CheckupTask()