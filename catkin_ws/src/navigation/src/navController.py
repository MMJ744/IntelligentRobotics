#!/usr/bin/env python
import rospy
from navigation.msg import Target
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

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


def main():
    global locations
    goalPub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=10)
    tbl1 = PoseStamped()
    tbl1.pose.position.x = 10
    tbl1.pose.position.y = 20
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    tbl1.pose.orientation.x = quaternion[0]
    tbl1.pose.orientation.y = quaternion[1]
    tbl1.pose.orientation.z = quaternion[2]
    tbl1.pose.orientation.w = quaternion[3]
    locations['table1'] = tbl1
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
