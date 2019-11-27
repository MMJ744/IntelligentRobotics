#!/usr/bin/env python
import rospy
from navigation.msg import Target

target = None

def inCallBack(data):
    global target
    target = data


def main():
    rospy.Subscriber("navInput", Target, inCallBack)
    rospy.init_node('navController', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target is None:
            rate.sleep()
            continue
        #Do stuff in here
        print(target.id)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
