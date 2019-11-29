#!/usr/bin/env python
import rospy
from navigation.msg import Target
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
import tf


target = None
locations = {}
completion = -1
goalPub = 0
def inCallBack(data):
    global target
    target = data


def navDoneCallback(data):
    global completion
    completion = data


def navigateTo(destination):
    global goalPub
    global completion
    rate = rospy.Rate(10)
    completion = -1
    if destination in locations:
        goalPub.publish(locations[destination])
        while completion == -1:
            rate.sleep()
        return completion #returns the code so you know what happened
    else:
        return -1


def main():
    goalPub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=10)
    rospy.Subscriber("navReturn",Int8,navDoneCallback)
    rospy.Subscriber("navInput", Target, inCallBack)
    rospy.init_node('navController', anonymous=True)
    initLocations():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target is None:
            rate.sleep()
            continue
        #Do stuff in here
        print(target.id)


def initLocations():
    global locations
    boothY = 13.5
    boothZ = -0.7
    boothW = 0.7
    tbl = PoseStamped()
    tbl.pose.position.x = 22
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothY
    tbl.pose.orientation.w = boothW
    locations['table1'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 15.89
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    locations['table2'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 9.87
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    locations['table3'] = tbl


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
