#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def talker():
    value = False;
    pub = rospy.Publisher('/mw/search', Bool, queue_size = 1)
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(0.1) # 0.1hz
    while not rospy.is_shutdown():
        value = not value
        rospy.loginfo(value)
        pub.publish(value)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
