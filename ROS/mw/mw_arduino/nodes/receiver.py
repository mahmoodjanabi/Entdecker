#!/usr/bin/env python

import rospy
import os
import re
import serial
import subprocess
import sys
import threading
import time
from std_msgs.msg import Bool, Float32, Int8, Int16
from mw_rtkrcv.msg import GpsLocation
from mw_video.msg import ConeLocation

nodeLock = threading.Lock()

class Receiver():
    def __init__(self):
        self.node_name = "mwArduinoReceiver"

        rospy.init_node(self.node_name)

        self.serial_port = serial.Serial('/dev/ttyACM99', 115200)
        self.touch_pub = rospy.Publisher("/mw/touch", Int8, queue_size = 1)
        self.search_pub = rospy.Publisher("/mw/search", Bool, queue_size = 1)
        self.speed_pub = rospy.Publisher("/mw/speed", Float32, queue_size = 1)
        
        self.solution_sub = rospy.Subscriber("/solution", GpsLocation, self.solution_callback, queue_size = 1)
        self.wp_current_sub = rospy.Subscriber("/mavros/mission/current", Int16, self.wp_current_callback, queue_size = 1)
        self.wp_reached_sub = rospy.Subscriber("/mavros/mission/current", Int16, self.wp_reached_callback, queue_size = 1)
        self.cone_location_sub = rospy.Subscriber("/mw/cone_location", ConeLocation, self.cone_callback, queue_size = 1)
        self.last_display_update = 0.0
        self.fix = 0
        self.latitude = 0.0
        self.longitude = 0.0
        self.sats = 0
        self.wp_current = 0
        self.wp_reached = 0
        self.cone_heading = 0.0
        self.cone_distance = 0.0
        self.ros_num = 0
        self.ping_result = 1
        self.wap_result = 1

        rospy.loginfo("%s: Ready." % self.node_name)

    def run(self):
        while not rospy.is_shutdown():
            t = time.time();

            if t - self.last_display_update > 1.0:
                self.last_display_update = t

                led_string = "l "

                if self.fix == 0:
                    led_string += "255 0 0\n"
                elif self.fix == 1:
                    led_string += "0 90 90\n"
                elif self.fix == 2:
                    led_string += "0 140 0\n"
                elif self.fix == 3 or self.fix == 4:
                    led_string += "225 200 15\n"
                elif self.fix == 5:
                    led_string += "210 121 34\n"
                else:
                    led_string += "255 0 0\n"

                self.serial_port.write(led_string)
                self.serial_port.write("o0 %.7f\n" % self.latitude)
                time.sleep(0.040)

                self.serial_port.write("t0 %.5f\n" % self.longitude)
                self.serial_port.write("o1 SATS    %2d\n" % self.sats)
                time.sleep(0.040)

                self.serial_port.write("o2 ROS     %s\n" % self.ros_num)
                self.serial_port.write("o3 NET  %2d %2d\n" % (self.ping_result, self.wap_result))
                time.sleep(0.040)

                self.serial_port.write("t1 W%3d ->%3d\n" % (self.wp_reached, self.wp_current))
                time.sleep(0.040)

                d = 0.0
                h = 0.0

                nodeLock.acquire()
                try:
                   d = self.cone_distance
                   self.cone_distance = 0.0
                   h = self.cone_heading
                   self.cone_heading = 0.0
                finally:
                    nodeLock.release()

                if d > 0.05:
                    # time.sleep(0.040)
                    self.serial_port.write("t2 D %8.2f\n" % d)
                    time.sleep(0.040)
                    self.serial_port.write("t3 H %8.2f\n" % h)
                else:
                    # time.sleep(0.040)
                    self.serial_port.write("t2           \n")
                    time.sleep(0.040)
                    self.serial_port.write("t3           \n")

            try:
                l = self.serial_port.readline()
                if l:
                    l = l.rstrip()
                    # rospy.loginfo("%s: line=%s" % (self.node_name, l))
                    m = re.match('^([-0-9]+)$', l)
                
                    if m:
                        value = int(m.group(1))

                        # rospy.loginfo("%s: Receiver got %d" % (self.node_name, value))
                        self.touch_pub.publish(value)

                        if value > 0:
                            self.search_pub.publish(False)

                    m = re.match('^E ([-0-9]+)( ([-0-9]+))?$', l)

                    if m:
                        value = int(m.group(1))
                        # rospy.loginfo("%s: Receiver E got %d" % (self.node_name, value))
                        # Spped
                        # 80 ticks/wheel rotation,
                        # circumfence 0.638m
                        # every 0.1 seconds
                        if len(m.group(3)) > 0:
                            period = 0.001 * int(m.group(3))
                        else:
                            period = 0.1

                        s = 0.638 * (float(value) / 80) / period   # now in m/s

                        # rospy.loginfo("%s: Receiver E value= %d period= %f s= %f" % (self.node_name, value, period, s))
                        self.speed_pub.publish(s)
            except serial.serialutil.SerialException:
                rospy.loginfo("close serial")

        rospy.loginfo("%s: Done." % self.node_name)
        self.serial_port.close()

    def solution_callback(self, loc):
        self.latitude = loc.latitude
        self.longitude = loc.longitude
        self.sats = loc.sats
        self.fix = loc.fix
        return

    def wp_current_callback(self, wp):
        self.wp_current = wp.data
        return

    def wp_reached_callback(self, wp):
        self.wp_reached = wp.data
        return

    def cone_callback(self, loc):
        nodeLock.acquire()
        try:
            self.cone_heading = loc.heading
            self.cone_distance = loc.distance
        finally:
            nodeLock.release()

        return

class CheckThread(threading.Thread):
    def __init__(self, group = None, target = None, name = None,
                 args = (), kwargs = None, verbose = None):
        threading.Thread.__init__(self, group = group, target = target, name = name,
                                  verbose = verbose)
        self.args = args
        self.kwargs = kwargs
        rospy.loginfo("CheckThread: init done.")
        return 
                
    def run(self):
        
        rospy.loginfo("CheckThread: run.")

        r = self.args[0]
        while True:
            r.ros_num = subprocess.check_output(["/home/odroid/bin/ros_processes"]).rstrip()
            r.ping_result = subprocess.call(["/home/odroid/bin/wap_check"])
            r.wap_result = subprocess.call(["/home/odroid/bin/net_check"])

            time.sleep(30)
        

def main(args):
    try:
        r = Receiver()
        t = CheckThread(args = (r,))
        t.daemon = True
        t.start()
    
        r.run()

    except KeyboardInterrupt:
        rospy.loginfo("close")

if __name__ == '__main__':
    main(sys.argv)
    
