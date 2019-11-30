#!/usr/bin/env python
import rospy
from std_msgs.msg import String

pub = 0

def navigate(loc):
    global pub
    pub.publish(loc)

def main():
    global pub
    rospy.init_node('web', anonymous=True)
    pub = rospy.Publisher('navIn',String,queue_size=10)