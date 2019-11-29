#!/usr/bin/env python
import rospy
from std_msgs.msg import String

pub = 0

def call(loc):
    global pub
    pub.publish(loc)

def main():
    print('hi')
    global pub
    rospy.init_node('web', anonymous=True)
    pub = rospy.Publisher('navIn',String,queue_size=10)


if __name__ == '__main__':
    main()