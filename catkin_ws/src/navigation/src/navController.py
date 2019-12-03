#!/usr/bin/env python
import rospy
from navigation.msg import Target
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String, Header
import tf, math


target = None
locations = {}
goalPub = 0
navResult = -1
x = 0.0
y = 0.0
theta = 0.0


def navOutCallback(x):
    global navResult
    navResult = x


def navIn(destination):
    global goalPub
    global locations
    print("trying to go to " + destination.data)
    if destination.data in locations:
        print("found destination " + destination.data)
        goalPub.publish(locations[destination.data])


def odomCallback(data):
    global x
    global y
    global theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    theta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)


def navigateTo(destination):
    global goalPub
    global locations
    global x
    global y
    global theta
    print("trying to go to " + destination.data)
    if destination.data in locations:
        print("found destination " + destination.data)
        goal = locations[destination.data]
        goalPub.publish(goal)
        q = goal.pose.orientation
        gtheta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        distance = math.sqrt(abs(goal.pose.position.x - x) + abs(goal.pose.position.y-y))
        while distance > 0.5 or abs(gtheta - theta) > 0.2:
            rate.sleep()
            distance = math.sqrt(abs(goal.pose.position.x - x) + abs(goal.pose.position.y-y))


def main():
    global goalPub
    initLocations()
    rospy.init_node('navController', anonymous=True)
    goalPub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=10)
    rospy.Subscriber("navIn", String, navIn)
    rospy.Subscriber("navOut", Int8, navOutCallback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


def initLocations():
    global locations
    boothY = 14.5
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
    tbl = PoseStamped()
    tbl.pose.position.x = 5.129
    tbl.pose.position.y = 17.120
    tbl.pose.orientation.z = 0.69
    tbl.pose.orientation.w = 0.72
    tbl.header = header
    locations['kitchen'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 26.75
    tbl.pose.position.y = 20
    tbl.pose.orientation.z = 0.69
    tbl.pose.orientation.w = 0.72
    tbl.header = header
    locations['frontdesk'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 26.75
    tbl.pose.position.y = 13
    tbl.pose.orientation.z = -0.5
    tbl.pose.orientation.w = 0.85
    tbl.header = header
    locations['waitingarea'] = tbl

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
