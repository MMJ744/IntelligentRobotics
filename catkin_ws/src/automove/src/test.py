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
    factor = 1.5
    while not rospy.is_shutdown():

        base_data = Twist()
        base_data.linear.x = 0.2 * factor
        if front < 0.5:
            print("FRONT " + str(front))
            base_data.linear.x = 0.0
            base_data.angular.z = -0.1 * factor
        if left < 0.6:
            print("LEFT " + str(left))
            base_data.angular.z = -0.1 * factor
        if right < 0.6:
            print("RIGHT " + str(right))
            base_data.angular.z = 0.1 * factor
        pub.publish(base_data)
        rate.sleep()


def extract_min_range(values, data):
    values = filter(lambda val: data.range_min < val < data.range_max and not math.isnan(val), values)
    if not values:
        return math.nan
    return min(values)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
