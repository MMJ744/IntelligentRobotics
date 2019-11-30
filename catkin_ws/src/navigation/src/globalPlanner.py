#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
from sensor_msgs.msg import LaserScan,PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
import tf

class Node():
    def __init__(self, x=0, y=0, parent=None, cost=1):
        self.parent = parent
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.f = 0
        self.cost = cost
    def __eq__(self,other):
        return self.x == other.x and self.y == other.y

costmap = OccupancyGrid()
width = 10
origin = (0,0)
height = 10
resolution = 1.0
cutoff = 100
pub = 0
go = False
goalLocation = (0,0)
currentLocation = (0,0)

def valid(x,y):
    global costmap
    global width
    global cutoff
    return costmap[y*width + x] < cutoff


def getNeighbours(n):
    global costmap
    global height
    global width
    neighbours = []
    diagcost = 1.5
    x = n.x
    y = n.y
    if x > 0:
        if valid(x-1,y):
            neighbours.append(Node(x-1,y))
        if y > 0:
            if valid(x-1,y-1):
                neighbours.append(Node(x-1,y-1, cost=diagcost))
        if y < height-1 and valid(x-1,y+1):
            neighbours.append(Node(x-1,y+1, cost=diagcost))
    if x < width-1:
        if y > 0 and valid(x+1,y-1):
            neighbours.append(Node(x+1,y-1, cost=diagcost))
        if y < height-1 and valid(x+1,y+1):
            neighbours.append(Node(x+1,y+1, cost=diagcost))
        if valid(x+1,y):
            neighbours.append(Node(x+1,y))
    if y > 0 and valid(x,y-1):
        neighbours.append(Node(x,y-1))
    if y < height-1 and valid(x,y+1):
        neighbours.append(Node(x,y+1))
    return neighbours


def h(current, goal):
    return (current.x-goal.x)**2 + (current.y-goal.y)**2


def findPath(startx, starty, goalx, goaly):
    if not (valid(startx,starty) and valid(goalx,goaly)):
        print("start or goal not in map")
        return None
    path = []
    openList = []
    closedList = []
    startNode = Node(startx, starty)
    goalNode = Node(goalx, goaly)
    openList.append(startNode)
    while(len(openList)>0):
        current = openList[0]
        index = 0
        for i,node in enumerate(openList): #Get smallest f value
            if node.f < current.f:
                current = node
                index = i
        openList.pop(index)
        closedList.append(current)
        if current==goalNode:
            while current is not None:
                pose = PoseStamped()
                pose.pose.position.x = current.x
                pose.pose.position.y = current.y
                path.append(pose)
                current = current.parent
            break
        for neighbour in getNeighbours(current):
            if neighbour in closedList:
                continue
            neighbour.g = current.g + neighbour.cost
            neighbour.h = h(neighbour, goalNode)
            neighbour.f = neighbour.g + neighbour.h
            neighbour.parent = current
            for n in openList:
                if n == neighbour and n.g <= neighbour.g:
                    continue
            openList.append(neighbour)
    return path[::-1]


def poseToMap(p):
    global resolution
    global origin
    x = int((p[0] - origin[0]) / resolution)
    y = int((p[1] - origin[1]) / resolution)
    return (x, y)


def mapToPose(p):
    x = int((p[0]*resolution) + origin[0])
    y = int((p[1]*resolution) + origin[1])
    return (x,y)


def test():
    global costmap
    costmap = [0, 0, 0, 0, 250, 0, 0, 0, 0, 0,0, 0, 0, 0, 250, 0, 0, 0, 0, 0,0, 0, 0, 0, 250, 0, 0, 0, 0, 0,0, 0, 0, 0, 250, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 250, 0, 0, 0, 0, 0,0, 0, 0, 0, 250, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 250, 0, 0, 0, 0, 0,0, 0, 0, 0, 250, 0, 0, 0, 0, 0]
    print(findPath(3,0,5,0))
    print(valid(4,0))
    print(valid(4,1))
    print(valid(4,5))


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
    origin = (data.origin.position.x,data.origin.position.y)


def poseCallback(data):
    global currentLocation
    currentLocation = poseToMap((data.pose.pose.position.x,data.pose.pose.position.y))


def goalCallback(data):
    global go
    global goalLocation
    goal = poseToMap((data.pose.position.x,data.pose.position.y))
    go = True


def main():
    global go
    rospy.init_node('Global_Planner', anonymous=True)
    rospy.Subscriber("move_base_node/global_costmap/costmap", OccupancyGrid, mapcallback)
    rospy.Subscriber('/map_metadata', MapMetaData, metaCallback)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber("move_base_simple/goal",PoseStamped, goalCallback)
    pathPub = rospy.Publisher("/move_base/GlobalPlanner/plan", Path, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if go:
            go = False
            path = findPath(currentLocation[0],currentLocation[1],goalLocation[0],goalLocation[1])
            pathPub.publish(path)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass