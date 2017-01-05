#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

from datetime import datetime
import os
import sys
import time

class Shutdown():
    def __init__(self):
        self.node_name = "mwShutdown"
        self.shutdown_timer = -1

        rospy.init_node(self.node_name)

        self.search_sub = rospy.Subscriber("/mw/touch", Int8, self.touch_callback, queue_size = 1)
        
        rospy.loginfo("%s: Ready." % self.node_name)

        while not rospy.is_shutdown():
            if self.shutdown_timer != -1:
                diff = datetime.now() - self.shutdown_timer
                if diff.total_seconds() > 20:
                    rospy.loginfo("ShutdownThread diff = %f, shutting down" % diff.total_seconds())
                    
                    os.system("/usr/bin/sudo /sbin/shutdown -h now");
                    return
        
            time.sleep(1)

        rospy.loginfo("%s: Done." % self.node_name)
        return

    def touch_callback(self, touch):
        # rospy.loginfo("Got touch %d" % touch.data)
        if touch.data != 3:
            self.shutdown_timer = -1
        else:
            if self.shutdown_timer == -1:
                self.shutdown_timer = datetime.now()
                # rospy.loginfo("Got touch == 3")
        return

def main(args):
    try:
        s = Shutdown()

    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
