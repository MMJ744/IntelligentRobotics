#!/usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

right = 999
front = 999
left = 999
theta = 0.0
pub = 0

def rotate(rotation):
    goal = theta + math.radians(-rotation)
    if goal > 3.1:
	    goal = -3.1 + (goal-3.1)
    if goal < -3.1:
	    goal = 3.1 + (goal +3.1)
    movement = Twist()
    print("theta " + str(theta) + " goal " + str(goal))
    rate = rospy.Rate(10)
    while abs(goal-theta) > 0.15:
        print("theta " + str(theta) + " goal " + str(goal))
        movement.angular.z =- 0.5
        pub.publish(movement)
        rate.sleep()


def odomCallback(data):
    global theta
    q = data.pose.pose.orientation
    theta = math.atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.z * q.z)

def callback(data):
    global front
    global right
    global left
    ranges = data.ranges
    sim = False
    if sim:
        length = len(ranges) / 4
        back_right = 999
        right = extract_min_range(ranges[:length], data)
        front = extract_min_range(ranges[length:length * 3], data)
        left = extract_min_range(ranges[length * 3:], data)
        left_back = 999
    else:
        length = len(ranges) / 6
        back_right = min(ranges[:length])
        right = extract_min_range(ranges[length:length * 2], data)
        front = extract_min_range(ranges[length * 2:length * 4], data)
        left = extract_min_range(ranges[length * 4:length * 5], data)
        left_back = min(ranges[length * 5:])


def talker():
    rospy.Subscriber("base_scan", LaserScan, callback)
    rospy.Subscriber("/odom", Odometry, odomCallback)
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('Mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    factor = 1.6
    cutoff = 0.8
    counter = 0
    while not rospy.is_shutdown():
        frontblocked = front < cutoff
        leftblocked = left < cutoff
        rightblocked = right < cutoff
        base_data = Twist()
        base_data.linear.x = 0.5 * factor
        increase = False
        if not leftblocked:
            increase = True
            print("can go left")
            base_data.angular.z = 0.2 * factor
            base_data.linear.x = 0.2 * factor
        elif frontblocked:
            increase = True
            counter += 1
            print("rotating right")
            rotate(10)
        if frontblocked:
            increase = True
            counter += 1
            print("front blocked")
            base_data.linear.x = 0
        if increase == False:
            counter = 0
        if counter > 50:
            rotate(75)
            counter = 0
    	pub.publish(base_data)
    	rate.sleep()

def extract_min_range(values, data):
    values = filter(lambda val: data.range_min < val < data.range_max and not math.isnan(val), values)
    if not values:
        return 1000
    return min(values)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
