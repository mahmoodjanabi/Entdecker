#!/usr/bin/env python

import rospy
import sys
import threading

from std_msgs.msg import Bool, Int16, Int8
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Waypoint, WaypointList

class Receiver():
    def __init__(self):
        self.node_name = "mwMavrosReceiver"
        self.lock = threading.Lock()
        self.previous_waypoint = 0
        self.waypoint_list = None

        rospy.init_node(self.node_name)

        self.search_pub = rospy.Publisher("/mw/search", Bool, queue_size = 1)
        self.avoid_pub = rospy.Publisher("/mw/avoid_direction", Int8, queue_size = 1)
        self.sound_pub = rospy.Publisher("/mw/sound", Int8, queue_size = 1)
        self.waypoints_sub = rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self.waypoints_callback, queue_size = 1)
        self.current_sub = rospy.Subscriber("/mavros/mission/current", Int16, self.current_callback, queue_size = 1)
        
        rospy.loginfo("%s: Ready." % self.node_name)

    def waypoints_callback(self, wlp):
        self.lock.acquire()
        try:
            self.waypoint_list = wlp.waypoints
        finally:
            self.lock.release()
        
        # rospy.loginfo("waypoints_callback wpl= %s" % self.waypoint_list)

    def current_callback(self, wp):
        new_wp = wp.data

        self.lock.acquire()
        try:
            waypoint_list = list(self.waypoint_list)
        except TypeError:
            waypoint_list = []
        finally:
            self.lock.release()


        # rospy.loginfo("current_callback %d" % new_wp)

        if waypoint_list == None or len(waypoint_list) == 0:
            # rospy.loginfo("current_callback no waypoint list yet")

            return

        if new_wp > 1 and self.previous_waypoint != new_wp - 1:
            self.previous_waypoint = new_wp - 1

            rospy.loginfo("   current_wp= %s" % waypoint_list[new_wp])

            if self.previous_waypoint > 0 and waypoint_list[self.previous_waypoint].z_alt > 1000:
                rospy.loginfo("set search to true: wp= %d, prev= %d" % (new_wp, self.previous_waypoint))

                if waypoint_list[self.previous_waypoint].z_alt == 1010:
                    # Left
                    self.avoid_pub.publish(1)
                elif waypoint_list[self.previous_waypoint].z_alt == 1020:
                    # Right
                    self.avoid_pub.publish(2)
                elif waypoint_list[self.previous_waypoint].z_alt == 1030:
                    # Finish/Stop
                    self.avoid_pub.publish(3)

                self.search_pub.publish(True)
                self.sound_pub.publish(1)
            else:
                rospy.loginfo("set search to false: wp= %d, prev= %d" % (new_wp, self.previous_waypoint))
                self.search_pub.publish(False)
                self.avoid_pub.publish(0)
                self.sound_pub.publish(2)

def main(args):
    try:
        rec = Receiver()
        rospy.spin()

    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
