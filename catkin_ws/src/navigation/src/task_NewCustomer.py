#!/usr/bin/env python
from State import State
from StateMachine import StateMachine
from Speech import speech, listen
import navTo
import taskManager
from rfid import readCard
import utils
import rospy


class NavigateToFront(State):

    def run(self, instance):
        navTo.navigateTo("frontdesk")

    def next(self, instance, input):
        return AskBooking()


class AskBooking(State):

    def run(self, instance):
        speech("Do you have a booking?")
        response = listen()
        response = 'yes' # TODO remove me
        instance.addInput(response)

    def next(self, instance, input):
        #if 'yes' in input:
        #    return BookingDetails()
        #if 'no' in input:
        #    return AskGroupSize()
        #return UnknownAnswer()
        res = utils.getYesNo(input)
        if res=='yes':
            return  BookingDetails()
        if res=='no':
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
            answer = utils.getNum(input)
            if answer == '' or answer == ' ': 
                return UnknownAnswer()
            instance.group_size = int(answer)
            return CheckGroup()


class GuideToWaitingarea(State):

    def run(self, instance):
        speech("Please follow me to the waiting area")
        navTo.navigateTo("waitingarea")
        speech("I will come get you when your table is ready")
        instance.addInput('')
        # add a task to collect them after the time
        taskManager.new_task("CollectFromWaitingArea", table_number=None, delay=5, customerID=-1)
        r = rospy.Rate(1)
        r.sleep()
        instance.running = False


class GuideToTable(State):

    def run(self, instance):
        speech("Please follow me")
        navTo.navigateTo("table" + str(instance.group_table))
        speech("Please take a seat. Someone will be with you in a few minutes")
        instance.addInput('')
        cusID = instance.model.tables[instance.group_table-1]['customerID']
        print(cusID)
        taskManager.new_task("TakeOrder", table_number=instance.group_table, delay=1, customerID=cusID)
        r = rospy.Rate(1)
        r.sleep()
        instance.running = False


class GiveWaitingTime(State):

    def run(self, instance):
        speech("Your wait time is 5 minutes, is this okay")
        response = listen()
        instance.addInput(response)

    def next(self, instance, input):
        result = utils.getYesNo(input)
        if 'yes' == result:
            return GuideToWaitingarea()
        else:
            return Goodbye()


class Goodbye(State):

    def run(self, instance):
        speech("Sorry about that. Goodbye")
        instance.running = False


class BookingDetails(State):

    def run(self, instance):
        speech("Please present your card to validate your booking")
        instance.user = readCard()

    def next(self, instance, input):
        if instance.user is not None:
            #Check bookings if they have a booking
            if instance.user in instance.model.bookings:
                instance.group_size = instance.model.bookings[instance.user]
                del instance.model.bookings[instance.user]
                speech("Hi there, " + instance.user)
                return CheckGroup()
            speech("Sorry I couldn't find your booking")
            return AskGroupSize()
        else:
            print("That didnt work")
            return UnknownAnswer()


class CheckGroup(State):

    def run(self, instance):
        speech("Table for " + str(instance.group_size))
        print("group=" + str(instance.group_size))

    def next(self, instance, input):

        big_tables = list(filter(lambda t: t['places'] >= instance.group_size, instance.model.tables))
        print("big enough = " + str(big_tables))

        if big_tables == []:
            speech("Sorry, we don't have any tables in the restaurant big enough to seat " + str(instance.group_size) + " people.")
            return Goodbye()

        big_avail_tables = list(filter(lambda t: t['available'], big_tables))
        print("big enough and free = " + str(big_avail_tables))

        if big_avail_tables == []:
            speech("Sorry, all our tables of size " + str(instance.group_size) + " or above are busy at the moment.")
            return GiveWaitingTime()

        big_avail_tables.sort(key=(lambda x: x['places']))
        print("big enough and free and sorted = " + str(big_avail_tables))

        instance.group_table = big_avail_tables[0]['id']
        instance.model.tables[instance.group_table-1]['customerID'] +=1 
        instance.model.tables[instance.group_table-1]['avaliable'] = False
        return GuideToTable()


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
