#!/usr/bin/env python

import math
import sys
import time

import rospy
from sensor_msgs.msg import LaserScan

class lidarScanCombiner():
    def __init__(self):
        self.node_name = "mwLidarCombiner"
        self.time_micros = time.time()

        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        self.is_debug = rospy.get_param('~debug', False)
        rospy.loginfo('%s: Parameter %s has value %s', self.node_name, rospy.resolve_name('~debug'), self.is_debug)

        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 1)
        self.cscan_pub = rospy.Publisher("/mw/combined_scan", LaserScan, queue_size = 1)

        rospy.loginfo("%s: Ready." % self.node_name)

    def scan_callback(self, scan):
        my_scan = LaserScan()
        my_scan.header.stamp = rospy.Time.now()
        my_scan.header.frame_id = scan.header.frame_id
        combine_vales = 10
        inf = float("inf")

        my_scan.angle_min = scan.angle_min + (combine_vales / 2.0) * scan.angle_increment
        my_scan.angle_max = scan.angle_max
        my_scan.angle_increment = scan.angle_increment * combine_vales
        my_scan.time_increment = scan.time_increment
        my_scan.scan_time = scan.scan_time
        my_scan.range_min = scan.range_min
        my_scan.range_max = scan.range_max

        i_array = []
        r_array = []
        i_v = 0.0
        r_v = inf
        c = 0
        for i in range(len(scan.intensities)):
            if scan.intensities[i] > 0.0:
                if scan.intensities[i] > i_v:
                    i_v = scan.intensities[i]

                if scan.ranges[i] < r_v:
                    r_v = scan.ranges[i]

            c += 1
            if c == combine_vales:
                i_array.append(i_v)
                r_array.append(r_v)
                i_v = 0.0
                r_v = inf
                c = 0

        my_scan.intensities = i_array
        my_scan.ranges = r_array

        self.cscan_pub.publish(my_scan)
    
    def cleanup(self):
        print "Shutting down lidar combiner node."
    
def main(args):       
    try:
        lidarScanCombiner()
        rospy.spin()
    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
