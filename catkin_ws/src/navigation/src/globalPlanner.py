#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
from sensor_msgs.msg import LaserScan,PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path#, GetPlan
import tf

class Node():
    def __init__(self, x=0, y=0, parent=None):
        self.parent = parent
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self,other):
        return self.x == other.x and self.y == other.y

costmap = [[]]
width = 10
height = 10
resolution = 0.05

def valid(x,y):
    global costmap
    return costmap[x][y] == 0


def getNeighbours(n):
    global costmap
    global height
    global width
    neighbours = []
    x = n.x
    y = n.y
    if x > 0:
        if costmap[x-1][y]==0:
            neighbours.append(Node(x-1,y))
        if y > 0:
            if costmap[x-1][y-1]==0:
                neighbours.append(Node(x-1,y-1))
        if y < height-1 and costmap[x-1][y+1]==0:
            neighbours.append(Node(x-1,y+1))
    if x < width-1:
        if y > 0 and costmap[x+1][y-1]==0:
            neighbours.append(Node(x+1,y-1))
        if y < height-1 and costmap[x+1][y+1]==0:
            neighbours.append(Node(x+1,y+1))
        if costmap[x+1][y]==0:
            neighbours.append(Node(x+1,y))
    if y > 0 and costmap[x][y-1]==0:
        neighbours.append(Node(x,y-1))
    if y < height-1 and costmap[x][y+1]==0:
        neighbours.append(Node(x,y+1))
    return neighbours


def h(current, goal):
    return (current.x-goal.x)**2 + (current.y-goal.y)**2


def findPath(costmap, startx, starty, goalx, goaly):
    if(costmap[startx][starty]==1 or costmap[startx][starty==1]):
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
                path.append((current.x,current.y))
                current = current.parent
            break
        for neighbour in getNeighbours(current):
            if neighbour in closedList:
                continue
            neighbour.g = current.g + 1
            neighbour.h = h(neighbour, goalNode)
            neighbour.f = neighbour.g + neighbour.h
            neighbour.parent = current
            for n in openList:
                if n == neighbour and n.f <= neighbour.f:
                    continue
            openList.append(neighbour)
    return path[::-1]

def test():
    global costmap
    costmap = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    print(findPath(costmap,0,0,1,8))



if __name__ == '__main__':
    test()



