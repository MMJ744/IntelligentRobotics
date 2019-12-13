#!/usr/bin/env python
import rospy
from std_msgs.msg import String

pub = None
lis = None
dataN = None
def navCallback(data):
    global dataN
    dataN = data.data
    #print('navto callback')
    #print(dataN)

def navigateTo(loc):
    print('navto going ' + loc)
    global pub
    global dataN
    global lis
    dataN = ''
    rate = rospy.Rate(10)
    if pub is None:
        pub = rospy.Publisher('navIn',String,queue_size=10)
        lis = rospy.Subscriber('navOut', String, navCallback)
    rate.sleep()
    rate.sleep()
    pub.publish(loc)
    #print('navto i did it')
    #print(pub)
    rate = rospy.Rate(10)
    while dataN != "done":
        rate.sleep()
        if dataN == "invalid": return False
    dataN = ''
    print('HERE')
    rate = rospy.Rate(0.2)
    rate.sleep()
    return True
    

def navigate(loc):
    global pub
    pub.publish(loc)
