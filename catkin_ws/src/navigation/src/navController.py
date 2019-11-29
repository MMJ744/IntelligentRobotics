#!/usr/bin/env python
import rospy
from navigation.msg import Target
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String, Header
import tf


target = None
locations = {}
goalPub = 0

def navigateTo(destination):
    global goalPub
    global locations
    print("trying to go to " + destination.data)
    if destination.data in locations:
        print("found destination " + destination.data)
        goalPub.publish(locations[destination.data])


def main():
    global goalPub
    initLocations()
    rospy.init_node('navController', anonymous=True)
    goalPub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=10)
    rospy.Subscriber("navIn", String, navigateTo)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target is None:
            rate.sleep()
            continue
        #Do stuff in here
        print(target.id)
    print("main exited")


def initLocations():
    global locations
    boothY = 13.5
    boothZ = -0.7
    boothW = 0.7
    header = Header()
    header.frame_id = 'map'
    tbl = PoseStamped()
    tbl.pose.position.x = 22
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothY
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table1'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 15.89
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table2'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 9.87
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table3'] = tbl


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
