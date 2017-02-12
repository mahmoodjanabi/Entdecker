#!/usr/bin/env python

import sys
import time
import rospy
from std_msgs.msg import Bool, Int8, Int16
from mavros_msgs.msg import OverrideRCIn, ParamValue
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, WaypointSetCurrent

class TouchHandler():
    def __init__(self):
        self.node_name = "mwMavrosTouchHandler"
        self.avoid_direction = 0                    # 1 - Left, 2 - Right
        self.current_wp = -1
        self.touch_value = -1
        self.finished = False

        self.apm_rc1_trim = 0
        self.apm_rc3_trim = 0
        self.auto_kickstart = -1

        rospy.init_node(self.node_name)

        rospy.wait_for_service('/mavros/param/get')
        self.get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)

        rospy.wait_for_service('/mavros/param/set')
        self.set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 1)
        self.sound_pub = rospy.Publisher("/mw/sound", Int8, queue_size = 1)

        self.avoid_sub = rospy.Subscriber("/mw/avoid_direction", Int8, self.avoid_callback, queue_size = 1)
        self.current_sub = rospy.Subscriber("/mavros/mission/current", Int16, self.current_callback, queue_size = 1)
        self.touch_sub = rospy.Subscriber("/mw/touch", Int8, self.touch_callback, queue_size = 1)
        
        ret = None
        try:
            ret = self.get_param(param_id = 'RC1_TRIM')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc1_trim = ret.value.integer
        else:
            rospy.logerr("get_param(RC1_TRIM) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'RC3_TRIM')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc3_trim = ret.value.integer
        else:
            rospy.logerr("get_param(RC3_TRIM) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'AUTO_KICKSTART')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)
        
        if ret != None and ret.success:
            self.auto_kickstart = ret.value.real 
        else:
            rospy.logerr("get_param(AUTO_KICKSTART) request failed. Check mavros logs")

        rospy.loginfo("%s: Ready." % self.node_name)
        rospy.loginfo("   rc1_trim= %d  rc3_trim= %d" %
            (self.apm_rc1_trim, self.apm_rc3_trim))

    def avoid_callback(self, av):
        self.avoid_direction = av.data
        if av.data > 0:
            self.finished = False

    def current_callback(self, cp):
        self.current_wp = cp.data

    def touch_callback(self, tp):
        touch = tp.data

        if self.touch_value == touch:
            return

        if self.finished:
            return

        self.touch_value = touch
 
        if touch == 0:
            return

        rospy.loginfo("%s: Touch got %d - Set MANUAL" % (self.node_name, touch))

        # try:
        #     ret = self.set_mode(base_mode = 0, custom_mode = 'MANUAL')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if not ret.success:
        #     rospy.logerr("Request failed. Check mavros logs")

        orc = OverrideRCIn();
        orc.channels[1] = 0
        orc.channels[3] = 0
        orc.channels[4] = 0
        orc.channels[5] = 0
        orc.channels[6] = 0
        orc.channels[7] = 0

        rospy.loginfo("%s: Neutral" % self.node_name)
        orc.channels[0] = self.apm_rc1_trim                    # Steering straight
        orc.channels[2] = self.apm_rc3_trim                    # Neutral for a bit
        self.rc_override_pub.publish(orc);
        time.sleep(0.05)                                       # Let the ESC prepare for the reverse
 
        rospy.loginfo("%s: Brake" % self.node_name)
        orc.channels[2] = self.apm_rc3_trim - 100              # Brake for a bit
        self.rc_override_pub.publish(orc);
        time.sleep(0.3)                                          # Let the brake work for a bit

        rospy.loginfo("%s: Neutral" % self.node_name)
        orc.channels[2] = self.apm_rc3_trim                    # Neutral for a bit
        self.rc_override_pub.publish(orc);
        time.sleep(0.05)                                       # Let the ESC prepare for the reverse

        if self.avoid_direction == 3:                          # Finish/Stop
            self.finished = True
            orc.channels[0] = 0
            orc.channels[2] = 0
            self.rc_override_pub.publish(orc);                 # Release
            time.sleep(0.05)
            self.sound_pub.publish(0)

            return                                             # XXX: For now!
 
        rospy.loginfo("%s: Back off" % self.node_name)
        orc.channels[2] = self.apm_rc3_trim - 100 # 20150403 125              # Really back off
        self.rc_override_pub.publish(orc);
        time.sleep(2.5)

        rospy.loginfo("%s: Neutral" % self.node_name)
        orc.channels[2] = self.apm_rc3_trim                    # Neutral for a bit
        self.rc_override_pub.publish(orc);
        time.sleep(0.05)                                       # Let the ESC prepare for the reverse
 
        rospy.loginfo("%s: Turn forward" % self.node_name)
        if self.avoid_direction > 0:
            if self.avoid_direction == 1:
                orc.channels[0] = orc.channels[0] + 200
            else:
                orc.channels[0] = orc.channels[0] - 200
        else:
            if touch < 2:
                orc.channels[0] = orc.channels[0] - 200
            else:
                orc.channels[0] = orc.channels[0] + 200

        orc.channels[2] = self.apm_rc3_trim + 100              # Forward
        self.rc_override_pub.publish(orc);
        time.sleep(1.5)                                        # Turn forward

        rospy.loginfo("%s: Straight forward" % self.node_name)
        orc.channels[0] = self.apm_rc1_trim
        orc.channels[2] = self.apm_rc3_trim + 100
        self.rc_override_pub.publish(orc);                     # Straight & neutral
        time.sleep(1)

        rospy.loginfo("%s: Release" % self.node_name)

        orc.channels[0] = 0
        orc.channels[2] = 0
        self.rc_override_pub.publish(orc);                     # Release
        time.sleep(0.05)

        if self.current_wp > 1:
            rospy.loginfo("%s: continue AUTO current wp= %d" % (self.node_name, self.current_wp))

            ret = None
            try:
                ret = self.set_param(param_id = 'AUTO_KICKSTART',
                    value = ParamValue(integer = 0, real = 0.0))
            except rospy.ServiceException as ex:
                rospy.logerr(ex)
        
            if ret == None or not ret.success:
                rospy.logerr("set_param(AUTO_KICKSTART, 0.0) request failed. Check mavros logs")

            ret = None
            try:
                ret = self.set_mode(base_mode = 0, custom_mode = 'AUTO')
            except rospy.ServiceException as ex:
                rospy.logerr(ex)

            if not ret.success:
                rospy.logerr("set_mode(0, AUTO) failed. Check mavros logs")

            time.sleep(0.5)
            self.sound_pub.publish(2)

            ret = None
            try:
                ret = self.set_param(param_id = 'AUTO_KICKSTART',
                    value = ParamValue(integer = 0, real = self.auto_kickstart))
            except rospy.ServiceException as ex:
                rospy.logerr(ex)
        
            if ret == None or not ret.success:
                rospy.logerr("set_param(AUTO_KICKSTART, %f) request failed. Check mavros logs" % self.auto_kickstart)

def main(args):
    try:
        TouchHandler()
        rospy.spin()

    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
