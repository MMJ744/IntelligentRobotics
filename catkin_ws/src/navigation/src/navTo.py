#!/usr/bin/env python
import rospy
from std_msgs.msg import String

pub = 0
lis = 0
dataN = 0
def navCallback(data):
    global dataN
    dataN = data

def navigateTo(loc):
    global pub
    global dataN
    pub.publish(loc)
    rate = rospy.Rate(10)
    while dataN != "done":
        rate.sleep()
        if dataN == "invalid": return False
    dataN = ''
    return True
    

def navigate(loc):
    global pub
    pub.publish(loc)

def main():
    global pub
    global lis
    global data
    pub = rospy.Publisher('navIn',String,queue_size=1)
    lis = rospy.Subscriber('navOut', String, navCallback)