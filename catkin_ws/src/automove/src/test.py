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
	rotate_rate = math.pi/8
	rate = rospy.Rate(1)

	rotation_in_radians = math.radians(rotation_in_degrees)
	wait_time = rotation_in_radians / rotate_rate

	base_data.angular.z = rotate_rate
	rate.sleep(int(wait_time))
	base_data.angular.z = 0

def callback(data):
	ranges = data.ranges
	length = len(ranges) / 4
	right = min(ranges[:length])
	front = min(ranges[length:length*3])
	left = min(ranges[length*3:])

def talker():
	rospy.Subscriber("base_scan", LaserScan, callback)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
	rospy.init_node('Mover', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():

		base_data = Twist()
		#base_data.linear.x = 0.3
		rotate(90, rospy, base_data)
		
		if front < 3:
			base_data.linear.x = 0
		pub.publish( base_data )
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass






	


