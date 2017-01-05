#!/usr/bin/env python

import rospy
import serial
import sys
from std_msgs.msg import Bool, Int8

class Receiver():
    def __init__(self):
        self.node_name = "mwArduinoReceiver"

        rospy.init_node(self.node_name)

        serial_port = serial.Serial('/dev/ttyACM99', 115200)
        touch_pub = rospy.Publisher("/mw/touch", Int8, queue_size = 1)
        search_pub = rospy.Publisher("/mw/search", Bool, queue_size = 1)
        
        rospy.loginfo("%s: Ready." % self.node_name)

        while not rospy.is_shutdown():
            try:
                value = int(serial_port.readline())
                rospy.loginfo("%s: Receiver got %d" % (self.node_name, value))
                touch_pub.publish(value)

                if value > 0:
                    search_pub.publish(False)
            except serial.serialutil.SerialException:
                rospy.loginfo("close serial")

        rospy.loginfo("%s: Done." % self.node_name)
        serial_port.close()

def main(args):
    try:
        Receiver()

    except KeyboardInterrupt:
        rospy.loginfo("close")

if __name__ == '__main__':
    main(sys.argv)
    
