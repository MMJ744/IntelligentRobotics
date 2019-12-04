#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def terms():

    filepath = "/home/bianca/Desktop/Robot/darknet/Output.txt"
    strings = ""
    rospy.init_node('vision_sender', anonymous=True)
    pub = rospy.Publisher('vision', String, queue_size=10)
    rate = rospy.Rate(2) # 2hz

    while not rospy.is_shutdown():
        with open(filepath) as fp:
            line = fp.readline()
            while line:
                strings.append(".")
                strings.append(line)
                line = fp.readline()
        rospy.loginfo(strings)
        pub.publish(strings)
        rate.sleep()

##    filepath = "/home/bianca/Desktop/Robot/darknet/Output.txt"
##    strings = list()
##    with open(filepath) as fp:
##        line = fp.readline()
##        cnt = 1
##        while line:
#            print("Line {}: {}".format(cnt, line.strip()))
##            strings.append(line)
##            line = fp.readline()
##            cnt += 1
##    print(len(strings))
##    for x in strings:
##        print(x)


#    f= open(filepath,"r")
#    if f.mode == 'r':
#        contents =f.read()
#    print(contents)

if __name__ == '__main__':
    terms()
#    print("AAAAAAA")
