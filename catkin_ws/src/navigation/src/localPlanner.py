#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
from sensor_msgs.msg import LaserScan,PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
from std_msgs.msg import Int8
import tf

path = []
costmap = OccupancyGrid()
width = 10
origin = (0,0)
height = 10
resolution = 1.0
cutoff = 100

def mapcallback(data):
    global costmap
    costmap = data.data


def metaCallback(data):
    global height
    global width
    global resolution
    global origin
    height = data.height
    width = data.width
    resolution = data.resolution
    origin = (origin.position.x,origin.position.y)


def pathCallback(data):
    global path
    path = data.poses


def main():
    navOutPub = rospy.Publisher('navOut', Int8, queue_size=10)
    movePub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('Local_Planner', anonymous=True)
    rospy.Subscriber("move_base_node/local_costmap/costmap", OccupancyGrid, mapcallback)
    rospy.Subscriber('/map_metadata', MapMetaData, metaCallback)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber("move_base_simple/goal",PoseStamped, goalCallback)
    rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, pathCallback)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        while len(path) == 0:
            rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("err global planner")