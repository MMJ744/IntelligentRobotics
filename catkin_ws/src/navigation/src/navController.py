#!/usr/bin/env python
import rospy
from navigation.msg import Target
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int8, String, Header
import tf, math


target = None
locations = None
goalPub = 0
navResult = -1
x = 0.0
donePub = None
odomsub = None
y = 0.0
odomtheta = 0.0
amcltheta = 0.0
x2 = 0.0
y2 = 0.0

def navOutCallback(x):
    global navResult
    navResult = x


def navIn(destination):
    global goalPub
    global locations
    global locations
    global donePub
    global odomsub
    global x
    global y
    global x2
    global y2
    global odomtheta
    global amcltheta
    rate = rospy.Rate(10)
    destination = destination.data
    if odomsub is None or goalPub is None or locations is None:
        print("stuff was none")
        initLocations()
    print("trying to go to " + destination)
    if destination in locations:
        print("found destination " + destination)
        goal = locations[destination]
        rate.sleep()
        goalPub.publish(goal)
        q = goal.pose.orientation
        gtheta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)
        print("published goal")
        distance = math.sqrt(abs(goal.pose.position.x - x) + abs(goal.pose.position.y-y))
        d2 = math.sqrt(abs(goal.pose.position.x - x2) + abs(goal.pose.position.y-y2))
        while distance > 0.7 and d2 > 0.7 and not rospy.is_shutdown():
            print(distance)
            print(gtheta - odomtheta)
            print(gtheta - amcltheta)
            rate.sleep()
            distance = math.sqrt(abs(goal.pose.position.x - x) + abs(goal.pose.position.y-y))
            d2 = math.sqrt(abs(goal.pose.position.x - x2) + abs(goal.pose.position.y - y2))
        print("arrived")
        donePub.publish("done")
    else:
        print("Didnt match location")
        donePub.publish("invalid")
    print("bye")

def amclCallback(data):
    global x
    global y
    global amcltheta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    amcltheta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)


def odomCall(data):
    global odomtheta
    global x2
    global y2
    x2 = data.pose.pose.position.x
    y2 = data.pose.pose.position.y
    q = data.pose.pose.orientation
    odomtheta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)

def main():
    global goalPub
    global donePub
    global odomsub
    rospy.init_node('navController', anonymous=True)
    initLocations()
    odomsub = rospy.Subscriber("/odom", Odometry, odomCall)
    odomsub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amclCallback)
    goalPub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=10)
    rospy.Subscriber("navIn", String, navIn)
    donePub = rospy.Publisher("navOut", String, queue_size=1)
    rate = rospy.Rate(10)
    print("hi")
    while not rospy.is_shutdown():
        while True:
            rate.sleep()

def initLocations():
    global locations
    global odomsub
    global goalPub
    locations = {}
    boothY = 14.1
    boothZ = -0.7
    boothW = 0.7
    header = Header()
    header.frame_id = 'map'
    tbl = PoseStamped()
    tbl.pose.position.x = 22
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table1'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 15.89
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table3'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 19.0
    tbl.pose.position.y = boothY - 0.8
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table2'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 13
    tbl.pose.position.y = boothY - 0.8
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table4'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 9.87
    tbl.pose.position.y = boothY
    tbl.pose.orientation.z = boothZ
    tbl.pose.orientation.w = boothW
    tbl.header = header
    locations['table5'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 5.129
    tbl.pose.position.y = 19.120
    tbl.pose.orientation.z = 0.69
    tbl.pose.orientation.w = 0.72
    tbl.header = header
    locations['kitchen'] = tbl
    tbl = PoseStamped()
    tbl.pose.position.x = 26.75
    tbl.pose.position.y = 18.7
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
