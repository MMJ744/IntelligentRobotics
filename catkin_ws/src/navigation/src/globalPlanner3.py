#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion, PoseArray, Pose
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
from sensor_msgs.msg import LaserScan,PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
from std_msgs.msg import Header
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
class Node():
    def __init__(self, x=0, y=0, parent=None, cost=1.0):
        self.parent = parent
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.f = 0
        self.cost = cost
    def __eq__(self,other):
        return self.x == other.x and self.y == other.y

costmap = []
width = 10
origin = (0,0)
height = 10
arpub = 0
resolution = 1.0
cutoff = 5
pub = 0
go = False
goalLocation = (0,0)
goalPose = PoseStamped()
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
    diagcost = 1.4142135
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
    #return (abs(goal.x - current.x) + abs(goal.y - current.y))
    #return math.sqrt((current.x-goal.x)**2 + (current.y-goal.y)**2)
    dx = abs(goal.x - current.x)
    dy = abs(goal.y - current.y)
    return 1*(dx+dy) + (2-2*1)*min(dx,dy)


def findPath(startx, starty, goalx, goaly):
    global costmap
    #print(costmap)
    print("finding path")
    if not (valid(startx,starty) and valid(goalx,goaly)):
        print("start or goal not in map")
        return None
    openList = []
    closedList = []
    startNode = Node(startx, starty)
    goalNode = Node(goalx, goaly)
    startNode.h = h(startNode,goalNode)
    startNode.f = startNode.g + startNode.h
    openList.append(startNode)
    while(len(openList)>0):
        current = openList[0]
        index = 0
        for i,node in enumerate(openList): #Get smallest f value
            if node.f < current.f:
                current = node
                index = i
        current = openList.pop(index)
        closedList.append(current)
        if current==goalNode:
            path = []
            path.append(goalPose)
            current = current.parent
            while current is not None:
                pose = PoseStamped()
                header = Header()
                header.frame_id = "map"
                coord = mapToPose((current.x,current.y))
                pose.pose.position.x = coord[0]
                pose.pose.position.y = coord[1]
                pose.header = header
                path.append(pose)
                current = current.parent
            print("got path")
            posarr = PoseArray()
            header = Header()
            header.frame_id = "map"
            l = []
            for c in closedList:
                pose = Pose()
                p = mapToPose((c.x,c.y))
                pose.position.x = p[0]
                pose.position.y = p[1]
                l.append(pose)
            posarr.poses = l
            posarr.header = header
            print(posarr)
            arpub.publish(posarr)
            return path[::-1]
        neighbours =getNeighbours(current)
        for neighbour in neighbours:
            if neighbour in closedList:
                continue
            #neighbour.g = current.g + neighbour.cost
            neighbour.g = current.g + neighbour.cost
            neighbour.h = h(neighbour, goalNode)
            neighbour.f = neighbour.g + neighbour.h
            neighbour.parent = current
            if neighbour not in openList:
                openList.append(neighbour)
            else:
                i = openList.index(neighbour)
                if neighbour.g < openList[i].g:
                    openList[i].g = neighbour.g
                    openList[i].f = neighbour.f
                    openList[i].h = neighbour.h
                    openList[i].parent = current
    print("couldnt make path")
    return [startNode]


def poseToMap(p):
    global resolution
    global origin
    x = int((p[0] - origin[0]) / resolution)
    y = int((p[1] - origin[1]) / resolution)
    return (x, y)


def mapToPose(p):
    global origin
    x = float(p[0]*resolution + origin[0])
    y = float(p[1]*resolution + origin[1])
    return (x,y)


def test():
    global costmap
    costmap = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    path = findPath(0,0,9,7)
    for i in range(len(path)-1):
        p1 = path[i]
        p2 = path[i+1]
        theta = calculateAngle(p1.pose.position,p2.pose.position)
        q = quaternion_from_euler(0,0,theta)
        p1.pose.orientation = q
        path[i] = p1
    print(path)
    print(valid(4,0))
    print(valid(4,1))
    print(valid(4,5))


def mapcallback(data):
    global costmap
    #print('map call ' + str(len(data.data)))
    costmap = data.data


def metaCallback(data):
    global height
    global width
    global resolution
    global origin
    height = data.height
    width = data.width
    resolution = 0.050000
    origin = (data.origin.position.x,data.origin.position.y)


def poseCallback(data):
    global currentLocation
    currentLocation = poseToMap((data.pose.pose.position.x,data.pose.pose.position.y))


def goalCallback(data):
    global go
    global goalLocation
    global goalPose 
    goalPose = PoseStamped()
    goalPose.pose = data.pose
    header = Header()
    header.frame_id = "map"
    goalLocation = poseToMap((data.pose.position.x,data.pose.position.y))
    go = True


def calculateAngle(p1,p2):
    theta = math.atan2(p2.y-p1.y,p2.x-p1.x)*180/math.pi
    if theta < 0:
        theta += 360
    return theta


def main():
    global go
    global currentLocation
    global goalLocation
    global arpub
    rospy.init_node('Global_Planner', anonymous=True)
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, mapcallback)
    rospy.Subscriber('/map_metadata', MapMetaData, metaCallback)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber("move_base_simple/goal",PoseStamped, goalCallback)
    pathPub = rospy.Publisher("/move_base/GlobalPlanner/plan", Path, queue_size=10)
    arpub = rospy.Publisher("killme", PoseArray, queue_size=1)
    #pathPub = rospy.Publisher("/path", Path, queue_size=1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if go:
            go = False
            path = findPath(currentLocation[0],currentLocation[1],goalLocation[0],goalLocation[1])
            for i in range(len(path)-1):
                p1 = path[i]
                p2 = path[i+1]
                theta = calculateAngle(p1.pose.position,p2.pose.position)
                q = quaternion_from_euler(0,0,theta)
                quat = Quaternion()
                quat.x = q[0]
                quat.y = q[1]
                quat.z = q[2]
                quat.w = q[3]
                p1.pose.orientation = quat
                path[i] = p1
            plan = Path()
            plan.poses = path
            header = Header()
            header.frame_id = "map"
            plan.header = header
            #print(plan)
            pathPub.publish(plan)
            print("published path")
        rate.sleep()


if __name__ == '__main__':
    try:
        #test()
        main()
    except rospy.ROSInterruptException:
        pass