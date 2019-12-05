#!/usr/bin/env python
import rospy
import os

def BerNerNerNerNerNerNer():
    #Clear costmap
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    print("spooky")

def BerNerNerNerNerNer():
    rospy.init_node('GhostBuster',anonymous=True)
    rate = rospy.Rate(0.3)
    print("hi")
    while not rospy.is_shutdown():
        rate.sleep()
        BerNerNerNerNerNerNer()
    print("bye")



if __name__ == '__main__':
    try:
        BerNerNerNerNerNer()
    except rospy.ROSInterruptException:
        pass
