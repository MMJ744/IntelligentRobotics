#!/usr/bin/env python
import rospy
from navigation.msg import Target
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String
import tf


target = None
locations = {}
goalPub = 0

def navigateTo(destination):
    global goalPub
    if destination in locations:
        goalPub.publish(locations[destination])


def main():
    goalPub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=10)
    rospy.Subscriber("navInput", String, navigateTo)
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
