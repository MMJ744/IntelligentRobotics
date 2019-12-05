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
theta = 0.0


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
    global theta
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
        print(distance)
        print(gtheta - theta)
        while distance > 0.5 :
            rate.sleep()
            distance = math.sqrt(abs(goal.pose.position.x - x) + abs(goal.pose.position.y-y))
        print("arrived")
        donePub.publish("done")
    else:
        print("Didnt match location")
        donePub.publish("invalid")
    print("bye")

def odomCallback(data):
    global x
    global y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    

def odomCall(data):
    global theta
    q = data.pose.pose.orientation
    theta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)

def main():
    global goalPub
    global donePub
    global odomsub
    rospy.init_node('navController', anonymous=True)
    initLocations()
    odomsub = rospy.Subscriber("/odom", Odometry, odomCall)
    odomsub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, odomCallback)
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
