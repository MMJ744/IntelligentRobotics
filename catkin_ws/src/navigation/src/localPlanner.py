#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, PoseWithCovariance, TwistWithCovariance, Pose
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
from sensor_msgs.msg import LaserScan,PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
from std_msgs.msg import Int8
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler


path = []
costmap = []
width = 5
origin = (0,0)
height = 5
resolution = 1.0
cutoff = 100
x = 0
y = 0
theta = 0

def mapcallback(data):
    global costmap
    costmap = data.data


def metaCallback(data):
    global resolution
    resolution = data.resolution

def odomCallback(data):
    global x
    global y
    global theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    theta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)


def pathCallback(data):
    global path
    path = data.poses


def checkOpen(p):
    global resolution
    global origin
    global width
    global height
    global cutoff
    global x
    global y
    origin = (x-(width/2*resolution),y-(height/2*resolution))
    localx = int((p.pose.position.x - origin[0]) / resolution)
    localy = int((p.pose.position.y - origin[1]) / resolution)
    print(localx)
    print(localy)
    return costmap[localy*width + localx] < cutoff


def findNewRoute(goal):
    return []


def main():
    global x
    global y
    global theta
    global path
    navOutPub = rospy.Publisher('navOut', Int8, queue_size=10)
    movePub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('Local_Planner', anonymous=True)
    rospy.Subscriber("move_base/local_costmap/costmap", OccupancyGrid, mapcallback)
    rospy.Subscriber('/map_metadata', MapMetaData, metaCallback)
    rospy.Subscriber("odom", Odometry, odomCallback)
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, pathCallback)
    rate = rospy.Rate(10)
    blockage = False
    linearFactor = 1
    angluarFactor = 1
    twist = Twist()
    while not rospy.is_shutdown():
        rospy.wait_for_message('/move_base/GlobalPlanner/plan', Path)
        print('path recieved')
        while len(path) > 0:
            print('point')
            nextPoint = path.pop(0)
            if checkOpen(nextPoint):
                if blockage: #point inbetween was blocked must find new root
                    path = findNewRoute(nextPoint).extend(path)
                    nextPoint = path.pop(0)
                blockage = False
                #Go faster or slower depending on how far you are from goal
                distance = math.sqrt(((nextPoint.pose.position.x-x)**2) + ((nextPoint.pose.position.y-y)**2))
                while distance > 0.01: #keep going till u close enough
                    twist.linear.x = linearFactor * distance
                    q = nextPoint.pose.orientation
                    gtheta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)
                    twist.angular.z = angluarFactor * (gtheta - theta)
                    movePub.publish(twist)
                    rate.sleep()
                    distance = math.sqrt(((nextPoint.pose.position.x-x)**2) + ((nextPoint.pose.position.y-y)**2))
            else:
                blockage = True
        if blockage:
            navOutPub.publish(-1) #Goal was blocked /
        navOutPub.publish(0) #Done


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass