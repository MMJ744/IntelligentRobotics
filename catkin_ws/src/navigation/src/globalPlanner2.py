#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
from sensor_msgs.msg import LaserScan,PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path, MapMetaData
from std_msgs.msg import Header
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import matplotlib.pyplot as plt

costmap = []
width = 10
origin = (0,0)
height = 10
resolution = 0.05
cutoff = 10
pub = 0
go = False
goalLocation = (0,0)
goalPose = PoseStamped()

staticObstacles = []

def processCostMap(costmap):
    global staticObstacles
    for x in range(width):
        for y in range(height):
            if costmap[y*width + x] > cutoff:
                staticObstacles.append((x,y))

def h(start, goal):
		#Use Chebyshev distance heuristic
		D = 1
		D2 = 1
		dx = abs(start[0] - goal[0])
		dy = abs(start[1] - goal[1])
		return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

def get_neighbours(pos):
		n = []
		for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
			x2 = pos[0] + dx
			y2 = pos[1] + dy
			if x2 < 0 or x2 > 7 or y2 < 0 or y2 > 7:
				continue
			n.append((x2, y2))
		return n

def move_cost(a, b):
		for obstacle in staticObstacles:
			if b in obstacle:
				return 100 
		return 1 

def AStarSearch(start, end):
 
	G = {} #Actual movement cost to each position from the start position
	F = {} #Estimated movement cost of start to end going via this position
 
	G[start] = 0 
	F[start] = h(start, end)
 
	closedNodes = set()
	openNodes = set([start])
	cameFrom = {}
 
	while len(openNodes) > 0:
		current = None
		currentFscore = None
		for pos in openNodes:
			if current is None or F[pos] < currentFscore:
				currentFscore = F[pos]
				current = pos
 
		if current == end:
			path = [current]
			while current in cameFrom:
				current = cameFrom[current]
				path.append(current)
			path.reverse()
			return path, F[end]

		openNodes.remove(current)
		closedNodes.add(current)
 
		for neighbour in get_neighbours(current):
			if neighbour in closedNodes: 
				continue 
			candidateG = G[current] + move_cost(current, neighbour)
 
			if neighbour not in openNodes:
				openNodes.add(neighbour)
			elif candidateG >= G[neighbour]:
				continue
 
			cameFrom[neighbour] = current
			G[neighbour] = candidateG
			H = h(neighbour, end)
			F[neighbour] = G[neighbour] + H
 
	raise RuntimeError("A* failed to find a solution")

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

def display(path, cost):
    global staticObstacles
    print ("route" + path)
    print ("cost" + cost)
    plt.plot([v[0] for v in path], [v[1] for v in path])
    for obstacle in staticObstacles:
        plt.plot([v[0] for v in obstacle], [v[1] for v in obstacle])
    plt.xlim(-1,width)
    plt.ylim(-1,height)
    plt.show()
    plt.savefig('path.png')

def main():
    global go
    rospy.init_node('Global_Planner', anonymous=True)
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, mapcallback)
    rospy.Subscriber('/map_metadata', MapMetaData, metaCallback)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber("move_base_simple/goal",PoseStamped, goalCallback)
    pathPub = rospy.Publisher("/path", Path, queue_size=1)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if go:
            go = False
            path,cost = AStarSearch((currentLocation[0],currentLocation[1]),(goalLocation[0],goalLocation[1]))
            display(path,cost)
            for i in range(len(path)-1):
                p1 = toPose(path[i])
                p2 = toPose(path[i+1])
                theta = calculateAngle(p1.pose.position,p2.pose.position)
                q = quaternion_from_euler(0,0,theta)
                quat = Quaternion()
                quat.x = q[0]
                quat.y = q[1]
                quat.z = q[2]
                quat.w = q[3]
                p1.pose.orientation = quat
                path = p1
            plan = Path()
            plan.poses = path
            header = Header()
            header.frame_id = "map"
            plan.header = header
            pathPub.publish(plan)
            print("published path")
        rate.sleep()

def toPose(point):
    tempPose = PoseStamped()
    tempPose.pose.postion.x = point[0]
    tempPose.pose.postion.y = point[1]

    header = Header()
    header.frame_id = "map"

    tempPose.header = header
    return tempPose


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass