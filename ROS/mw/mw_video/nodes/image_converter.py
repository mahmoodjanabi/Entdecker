#!/usr/bin/env python

import cv2
import math
import numpy as np
import sys
import time

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from mw_video.msg import ConeLocation

class cvBridgeFindCone():
    def __init__(self):
        self.node_name = "mwFindCone"
        self.time_micros = time.time()
        self.processing_time = 0
        self.average_processing_time = 0
        self.average_time = 0
        self.count = 0
        self.distance = 0
        self.delta_heading = 0
        self.found = False
        # self.frame_height = 480
        # self.frame_width = 864
        self.frame_height = 360
        self.frame_width = 640

        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        self.is_debug = rospy.get_param('~debug', False)
        rospy.loginfo('%s: Parameter %s has value %s', self.node_name, rospy.resolve_name('~debug'), self.is_debug)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size = 1)
        self.cone_pub = rospy.Publisher("/mw/cone_location", ConeLocation, queue_size = 1)
        self.image_pub = None

        if self.is_debug:
            self.image_pub = rospy.Publisher("/mw/image_topic", Image, queue_size = 1)

        rospy.loginfo("%s: Ready." % self.node_name)

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        self.count = self.count + 1
        frame = None

        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        if frame == None:
            return

        # Process the frame using the process_image() function
        t = time.time()
        display_image = self.process_image(frame)
        self.processing_time += time.time() - t

        if self.is_debug: 
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(display_image, "bgr8"))
            except CvBridgeError, e:
                print e

        if self.found:
            self.found = False
            rospy.loginfo("%s: distance: %f  delta_heading: %f" % (self.node_name, self.distance, self.delta_heading))
            cl = ConeLocation(heading = self.delta_heading, distance = self.distance)
            self.cone_pub.publish(cl)

        if self.count % 100 == 0:
            current_micros = time.time()
            self.average_time =(current_micros  - self.time_micros) / 100
            self.average_processing_time = self.processing_time / 100
            self.processing_time = 0
            rospy.loginfo("%s: time: %f  %f\n" % (self.node_name, self.average_time, self.average_processing_time))
            self.time_micros = current_micros

    def process_image(self, frame):
        blurMat = cv2.blur(frame, (3,3))
        hsv = cv2.cvtColor(blurMat, cv2.COLOR_BGR2HSV)

        # thresholded = cv2.inRange(hsv, (0, 190, 125, 0), (15, 255, 230, 0))
        # thresholded2 = cv2.inRange(hsv, (165, 190, 125, 0), (180, 255, 230, 0))
        # 20150402 thresholded = cv2.inRange(hsv, (0, 210, 125, 0), (10, 255, 255, 0))
        # 20150402 thresholded2 = cv2.inRange(hsv, (170, 210, 125, 0), (180, 255, 255, 0))
        thresholded = cv2.inRange(hsv, (0, 170, 125, 0), (10, 255, 255, 0))
        thresholded2 = cv2.inRange(hsv, (170, 170, 125, 0), (180, 255, 255, 0))
        # thresholded = cv2.inRange(hsv, (0, 210, 125, 0), (25, 255, 255, 0))
        # thresholded2 = cv2.inRange(hsv, (155, 210, 125, 0), (180, 255, 255, 0))

        thresholded = cv2.bitwise_or(thresholded, thresholded2)

        ret = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = None

        if len(ret) > 2:
            contours = ret[1]
        elif len(ret) == 2:
            contours = ret[0]

        if self.is_debug:
            t = "Avg Time: %6.4f (%6.4f)" % (self.average_time, self.average_processing_time)

        if contours != None:
            areas = [cv2.contourArea(c) for c in contours]

            if areas and len(areas) > 0:
                max_index = np.argmax(areas)
                cnt = contours[max_index]

                poly = cv2.approxPolyDP(cnt, 3, True)
                x,y,w,h =  cv2.boundingRect(poly)

                if w * h > 160 and w / h < 1.0:
                    self.distance = 0.24 * self.frame_height / (0.3969 * h)
                    self.delta_heading = (70.42 / self.frame_width) * ((x + w / 2) - (self.frame_width / 2))
                    self.found = True

                    if self.is_debug:
                        cv2.drawContours(frame, [poly], 0, (12, 236, 160), 1)
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (12, 236, 160), 2)
                        t += " Dist: %6.2fm   Head: %6.2f" % (self.distance, self.delta_heading)

        if self.is_debug:
            cv2.putText(frame, t, (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 60, 245), 2)

        return frame
    
    def cleanup(self):
        print "Shutting down vision node."
    
def main(args):       
    try:
        cvBridgeFindCone()
        rospy.spin()
    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
