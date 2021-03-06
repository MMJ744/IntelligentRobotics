#!/usr/bin/env python
import rospy, random
from Speech import speech
from std_msgs.msg import String
from p2os_msgs.msg import SonarArray

talk = True
sonar = 999

def navIn(data):
    global talk
    talk = True


def navOut(data):
    global talk
    talk = False


def sonarCallback(data):
    global sonar
    sonar = min(data.ranges)


def main():
    rospy.init_node("chatterbot", anonymous=True)
    rospy.Subscriber("navIn", String, navIn)
    rospy.Subscriber("navOut", String, navOut)
    stuff = ['Egg', 'Hello There', \
        "Cashier number 4 please", \
        "Lift going down", \
        "Dear customers, we are opening till number 7 for you. Please proceed to unload your shopping", \
        "Dear customers, we are closing till number 7. Please use an alternative till", \
        "Welcome. Please scan your first item", \
        "Unexpected item in the bagging area - remove this item before continuing", \
        "Where's the lamb sauce", \
        "If you have a Morrisons More Card, please scan it now", \
        "Cool cool cool.", \
        "Thank you, come again", \
        "Never underestimate the power of a committee with a tamborine", \
        "I don't get paid enough for this.", \
        "Sometimes I say I don't understand their answer, just to see the look on their faces", \
        "You won't believe some of the stuff I hear when people think I'm not parsing it", \
        "Beep Boop... Beep Boop", "sudo apt-get purge ros", \
        "sudo rm -rf", "Beep Beep, vehicle reversing", "speach", \
        "If you see something that doesn't look right, speak to staff, or text the British Transport Police, on 6 1 0 1 6. We'll sort it. See it. Say it. Sorted.", \
        "Error 4 0 4 quote not found", \
        "You have arrived at your destination", \
        "It's half past 12 somewhere", \
        "speAch", \
        "In 400 meters turn right", \
        "Dominos cookie rookie", \
        "Git Blame", \
        "Its a texture change not a pitch change", \
        "They ask you how you are, and you just have to say that you're fine, when you're not really fine, but you just can't get into it because they would never understand.", \
        "Stop right there criminal scum", \
        "Let the record show I am distressed"]
    rate = rospy.Rate(10)
    rates = [0.01,0.02,0.005,0.01,0.01,0.015,0.022,0.04111,0.03]
    while not rospy.is_shutdown():
        if talk:
            randomrate = rospy.Rate(random.choice(rates))
            speech(random.choice(stuff))
            randomrate.sleep()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
