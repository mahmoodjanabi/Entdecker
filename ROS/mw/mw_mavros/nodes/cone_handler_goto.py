#!/usr/bin/env python

import cv2
import math
import numpy as np
import sys
import threading
import time

import rospy
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge, CvBridgeError
from mavros.msg import Waypoint
from mavros.srv import CommandLong, SetMode, WaypointGOTO
from mw_video.msg import ConeLocation

class ConeHandler():
    def __init__(self):
        self.node_name = "mwConeHandler"
        self.lock = threading.Lock()
        self.heading = 0
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.last_target_lat = 0
        self.last_target_long = 0
        self.target_speed = 100

        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        rospy.wait_for_service('/mavros/cmd/command')
        self.set_command = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        rospy.wait_for_service('/mavros/mission/goto')
        self.set_goto = rospy.ServiceProxy('/mavros/mission/goto', WaypointGOTO)

        self.search_sub = rospy.Subscriber("/mw/search", Bool, self.search_callback, queue_size = 1)
        self.search_sub = rospy.Subscriber("/mw/cone_location", ConeLocation, self.cone_callback, queue_size = 1)
        self.heading_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_callback, queue_size = 1)
        self.navsat_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.navsat_callback, queue_size = 1)
        
        rospy.loginfo("%s: Ready." % self.node_name)

    def compass_callback(self, heading):
        self.lock.acquire()

        try:
            self.heading = heading.data
        finally:
            self.lock.release()

    def search_callback(self, bp):
        if bp.data:
            self.target_speed = 100                             # Set an initial target
            self.last_target_lat = 0
            self.last_target_long = 0


    def navsat_callback(self, navsat):
        self.lock.acquire()

        try:
            if navsat.status >= 0:
                self.latitude = navsat.latitude
                self.longitude = navsat.longitude
                self.altitude = navsat.altitude
            else:
                self.latitude = 0
                self.longitude = 0
                self.altitude = 0
        finally:
            self.lock.release()

    def cone_callback(self, cp):
        d = cp.distance
        d_hdg = cp.heading
        rospy.loginfo("distance: %f  delta_heading: %f" % (d, d_hdg))

        if self.latitude != 0:
            self.lock.acquire()
            try:
                new_heading = (self.heading + d_hdg + 360) % 360
                R = 6371000
                # d = d + 2  # one meter behind the real target
                new_lat = math.degrees(math.asin(math.sin(math.radians(self.latitude)) * math.cos(d / R) +
                    math.cos(math.radians(self.latitude)) * math.sin(d / R) * math.cos(math.radians(new_heading))))
                new_long = self.longitude + math.degrees(math.atan2(math.sin(math.radians(new_heading)) *
                    math.sin(d / R) * math.cos(math.radians(self.latitude)),
                    math.cos(d / R) - math.sin(math.radians(self.latitude)) * math.sin(math.radians(new_lat))));

                if abs(self.last_target_lat - new_lat) > 0.000001 or abs(self.last_target_long - new_long) > 0.000001:
                    rospy.loginfo("       lat: %f  long: %f  alt: %f" % (self.latitude, self.longitude, self.altitude))
                    rospy.loginfo("Target lat: %f  long: %f  alt: %f" % (new_lat, new_long, 0))

                    wp = Waypoint(frame = Waypoint.FRAME_GLOBAL, command = Waypoint.NAV_WAYPOINT,
                        autocontinue = True, param3 = 1.0, x_lat = new_lat, y_long = new_long, z_alt = 100.0)

                    ret = None
                    try:
                        ret = self.set_goto(waypoint = wp)
                    except rospy.ServiceException as ex:
                        rospy.logerr(ex)

                    if ret != None and not ret.success:
                        rospy.logerr("set_goto(%s) request failed. Check mavros logs" % wp)

                t_speed = 100
                if d < 4:
                    t_speed = 0.15
                elif d < 6:
                    t_speed = 0.15
                elif d < 8:
                    t_speed = 0.15

                if self.target_speed > t_speed:
                    rospy.loginfo("Change target speed %f -> %f" %(self.target_speed, t_speed))

                    self.target_speed = t_speed

                    ret = None
                    try:
                        ret = self.set_command(command = 178,                  # CommandLong.CMD_DO_CHANGE_SPEED,
                            param1 = 1, param2 = self.target_speed, param3 = -1)
                    except rospy.ServiceException as ex:
                        rospy.logerr(ex)

                    if ret != None and not ret.success:
                        rospy.logerr("set_command() request failed. Check mavros logs")

            finally:
                self.lock.release()
 
    def cleanup(self):
        rospy.loginfo("Shutting down vision node.")
    
def main(args):       
    try:
        ConeHandler()
        rospy.spin()
    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
