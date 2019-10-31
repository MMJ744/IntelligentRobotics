#!/usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

right = 999
front = 999
left = 999


def rotate(rotation_in_degrees, rospy, base_data):
    rotate_rate = math.pi / 8
    rate = rospy.Rate(1)

    rotation_in_radians = math.radians(rotation_in_degrees)
    wait_time = rotation_in_radians / rotate_rate

    base_data.angular.z = rotate_rate
    rate.sleep()
    base_data.angular.z = 0


def callback(data):
    global front
    global right
    global left
    ranges = data.ranges
    sim = True
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
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.init_node('Mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    factor = 2
    cutoff = 0.5
    while not rospy.is_shutdown():
        frontblocked = front < cutoff
        leftblocked = left < cutoff
        rightblocked = right < cutoff
        base_data = Twist()
        base_data.linear.x = 0.3 * factor
        if not leftblocked:
            print("can go left")
            base_data.angular.z = 0.12 * factor
            base_data.linear.x = 0.07 * factor
        elif frontblocked and not rightblocked:
            print("rotating right")
            base_data.angular.z = -1.5 * factor
        if frontblocked:
            print("front blocked")
            base_data.linear.x = 0
        if frontblocked and leftblocked and rightblocked:
            print("reversing")
            base_data.linear.x = -0.8 * factor
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
