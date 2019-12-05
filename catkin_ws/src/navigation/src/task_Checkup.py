from StateMachine import StateMachine
from State import State
from Speech import speech, listen
import navTo


class NavigateToTable(State):

    def run(self, instance):
        navTo.navigateTo("table" + str(instance.table))

    def next(self, instance, input):
        return PerformCheckup()


class PerformCheckup(State):

    def run(self, instance):
        speech("Is everything okay with your food?")
        instance.addInput(listen())

    def next(self, instance, input):
        if input == "":
            return UnknownAnswer()
        else:
            instance.model.prepend_message("staff", "Log checkup comment: \'" + input + "\' on table " + str(instance.table))
            return ConfirmComments()


class ConfirmComments(State):

    def run(self, instance):
        speech("Okay, thank you for your comments. They have been submitted to my organic comrades for review")
        instance.running = False


class UnknownAnswer(State):
    def run(self, instance):
        speech("Sorry, I didn't understand your answer, let me try again")
        instance.addInput('')

    def next(self, instance, input):
        return instance.previousState


class CheckupTask(StateMachine):
    def __init__(self, model, table):
        StateMachine.__init__(self, NavigateToTable(), model)
        self.table = table

# CheckupTask.navigateToTable = NavigateToTable()
# CheckupTask.performCheckup = PerformCheckup()
# CheckupTask.confirmComments = ConfirmComments()
# CheckupTask.unknownAnswer = UnknownAnswer()
#
# instance = CheckupTask()
# instance.runAll()


import taskExecuter
