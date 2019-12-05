#!/usr/bin/env python

import rospy
from std_msgs.msg import String

checkarray = [0, 0, 0, 0, 0]
counter = 0
sub = None

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    arepeoplearray(data)
    
def arepeoplearray(data):
    global counter
    global checkarray
    if "person" in data.data:
        checkarray[counter] = 1
    else:
        checkarray[counter] = 0
    counter = (counter + 1) % 5

def are_people():
    return 1 in checkarray

def listenpeople():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listenpeople', anonymous=True)

    global sub

    if sub is None:
        sub = rospy.Subscriber('vision', String, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    listenpeople()


listenpeople()