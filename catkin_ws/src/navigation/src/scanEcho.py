#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
pub = 0


def callback(data):
    global pub
    pub.publish(data)


def talker():
    global pub
    rospy.init_node('scanEcho', anonymous=True)
    pub = rospy.Publisher('scan',LaserScan, queue_size=10)
    rospy.Subscriber("base_scan", LaserScan, callback)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
	try:
		talker()
      	except rospy.ROSInterruptException:
        	pass
