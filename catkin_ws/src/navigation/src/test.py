#!/usr/bin/env python
import rospy
from navigation.msg import Target
from geometry_msgs.msg import PoseStamped
def talker():
	pub = rospy.Publisher('navInput', Target, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1) # 10hz
	c = 0
	while not rospy.is_shutdown():
		c+=1
		if c%3 == 0:
			pose = PoseStamped()
			pose.pose.position.x = c
			target = Target()
			target.id = c
			target.pose = pose
			pub.publish(target)
		rate.sleep()
  
if __name__ == '__main__':
	try:
		talker()
    except rospy.ROSInterruptException:
        pass

