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
    sonar = min(data.ranges[3:6])


def main():
    rospy.init_node("scream", anonymous=True)
    rospy.Subscriber("navIn", String, navIn)
    rospy.Subscriber("navOut", String, navOut)
    rospy.Subscriber("sonar", SonarArray, sonarCallback)
    rate = rospy.Rate(10)
    screamRate = rospy.Rate(0.75)
    while not rospy.is_shutdown():
        if talk and sonar < 0.3:
            speech("wilhelmscream")
            screamRate.sleep()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
