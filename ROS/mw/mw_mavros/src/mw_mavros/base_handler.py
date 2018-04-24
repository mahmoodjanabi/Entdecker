#!/usr/bin/env python

import cv2
from datetime import datetime
import math
import numpy as np
import sys
import threading
import time

import rospy
from std_msgs.msg import Bool, Float32, Float64, Int8
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import OverrideRCIn, ParamValue, RCOut, State, Waypoint
from mavros_msgs.srv import ParamGet, ParamSet, SetMode
from mw_video.msg import ConeLocation
from geometry_msgs.msg import TwistStamped

NaN = float('nan')
RC = 1.0 / (2.0 * math.pi * 20.0)

class PID_Info:
    P = 0.0
    I = 0.0
    D = 0.0
    desired = 0.0

class PID:
    _Kp = 0.0
    _Ki = 0.0
    _Kd = 0.0
    _integrator = 0.0
    _last_derivative = 0.0
    _last_error = 0.0
    _last_t = 0
    _imax = 5000.0
    _pid_info = PID_Info()

    def __init__(self, p, i, d):
        self._Kp = p
        self._Ki = i
        self._Kd = d

    def reset_i(self):
        self._integrator = 0
        # we use NAN (Not A Number) to indicate that the last
        # derivative value is not valid
        self._last_derivative = NaN
        self._pid_info.I = 0.0

    def get_pid(self, error, scaler = 1.0):
        error = float(error)
        tnow = time.clock() * 1000
        dt = tnow - self._last_t

        if self._last_t == 0 or dt > 1000:
            dt = 0

            # if this PID hasn't been used for a full second then zero
            # the intergator term. This prevents I buildup from a
            # previous fight mode from causing a massive return before
            # the integrator gets a chance to correct itself
            self.reset_i()

        self._last_t = tnow
        delta_time = float(dt) / 1000.0

        # Compute proportional component
        self._pid_info.P = error * self._Kp;
        output = self._pid_info.P

        # Compute derivative component if time has elapsed
        if abs(self._Kd) > 0 and dt > 0:
            derivative = 0.0

            if math.isnan(self._last_derivative):
                # we've just done a reset, suppress the first derivative
                # term as we don't want a sudden change in input to cause
                # a large D output change
                derivative = 0.0
                self._last_derivative = 0.0
            else:
                derivative = (error - self._last_error) / delta_time

            # discrete low pass filter, cuts out the
            # high frequency noise that can drive the controller crazy
            derivative = self._last_derivative + (delta_time / (RC + delta_time)) * (derivative - self._last_derivative)

            # update state
            self._last_error = error
            self._last_derivative = derivative

            # add in derivative component
            self._pid_info.D = self._Kd * derivative
            output += self._pid_info.D

        # scale the P and D components
        output *= scaler
        self._pid_info.D *= scaler
        self._pid_info.P *= scaler

        # Compute integral component if time has elapsed
        if abs(self._Ki) > 0 and dt > 0:
            self._integrator += (error * self._Ki) * scaler * delta_time

            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax

            self._pid_info.I = self._integrator
            output += self._integrator

        self._pid_info.desired = output

        return output

class BaseHandler(object):
    node_name = ""

    def __init__(self):
        self.nodeLock = threading.Lock()
        self.conditionVariable = threading.Condition(self.nodeLock)
        self.updates = 0

        self.arduino_speed_value = 0.0
        self.avoid_direction = 0                    # 1 - Left, 2 - Right, 3 - Stop
        self.fcu_mode = None
        self.is_manual = False
        self.manual_start_time = None
        self.touch_value = 0

        self.apm_rc1_max = 0
        self.apm_rc1_min = 0
        self.apm_rc1_reversed = 0
        self.apm_rc1_trim = 0

        self.apm_rc3_trim = 0
        self.apm_rc3_max = 0
        self.apm_rc3_min = 0
        self.apm_rc3_trim = 0
        self.apm_braking_percent = 30.0
        self.last_rc3_raw = 0

        self.throttle_pid = None
        self.last_throttle = 0

        rospy.init_node(self.node_name)

        self.is_debug = rospy.get_param('~debug', False)
        rospy.loginfo('%s: Parameter %s has value %s', self.node_name, rospy.resolve_name('~debug'), self.is_debug)
        
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.wait_for_service('/mavros/param/get')
        self.get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)

        rospy.wait_for_service('/mavros/param/set')
        self.set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 1)
        self.search_pub = rospy.Publisher("/mw/search", Bool, queue_size = 1)
        self.sound_pub = rospy.Publisher("/mw/sound", Int8, queue_size = 1)

        self.avoid_sub = rospy.Subscriber("/mw/avoid_direction", Int8, self.avoid_callback, queue_size = 1)
        self.arduino_speed_sub = rospy.Subscriber("/mw/speed", Float32, self.arduino_speed_callback, queue_size = 1)
        self.rcout_sub = rospy.Subscriber("/mavros/rc/out", RCOut, self.rcout_callback, queue_size = 1)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback, queue_size = 1)

        ret = None
        try:
            ret = self.get_param(param_id = 'RC1_MAX')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc1_max = ret.value.integer
        else:
            rospy.logerr("get_param(RC1_MAX) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'RC1_MIN')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc1_min = ret.value.integer
        else:
            rospy.logerr("get_param(RC1_MIN) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'RC1_REVERSED')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc1_reversed = ret.value.integer
        else:
            rospy.logerr("get_param(RC1_REVERSED) request failed. Check mavros logs")

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

        self.last_rc3_raw = self.apm_rc3_trim

        ret = None
        try:
            ret = self.get_param(param_id = 'RC3_MAX')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc3_max = ret.value.integer
        else:
            rospy.logerr("get_param(RC3_MAX) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'RC3_MIN')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc3_min = ret.value.integer
        else:
            rospy.logerr("get_param(RC3_MIN) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'RC3_TRIM')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc3_trim = ret.value.integer
        else:
            rospy.logerr("get_param(RC3_TRIM) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'BRAKING_PERCENT')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     self.apm_braking_percent = ret.value.integer
        # else:
        #     rospy.logerr("get_param(BRAKING_PERCENT) request failed. Check mavros logs")

        thr_p = 2.4 # 0.7
        thr_i = 0.2
        thr_d = 0.2
        thr_imax = 4000.0

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_SPEED_P')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     thr_p = ret.value.real
        # else:
        #     rospy.logerr("get_param(ATC_SPEED_P) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_SPEED_I')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     thr_i = ret.value.real
        # else:
        #     rospy.logerr("get_param(ATC_SPEED_I) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_SPEED_D')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     thr_d = ret.value.real
        # else:
        #     rospy.logerr("get_param(ATC_SPEED_D) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_SPEED_IMAX')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     thr_imax = ret.value.integer
        # else:
        #     rospy.logerr("get_param(ATC_SPEED_IMAX) request failed. Check mavros logs")

        self.throttle_pid = PID(thr_p, thr_i, thr_d)
        self.throttle_pid._imax = thr_imax

        rospy.loginfo("%s: Ready." % self.node_name)
        rospy.loginfo("   rc1 _max= %d  _min= %d  _rev= %d  _trim= %d" %
            (self.apm_rc1_max, self.apm_rc1_min, self.apm_rc1_reversed, self.apm_rc1_trim))
        rospy.loginfo("   rc3 _trim= %d" %
            (self.apm_rc3_trim))

    def arduino_speed_callback(self, tp):
        with self.conditionVariable:
            self.arduino_speed_value = tp.data
            self.updates += 1
            self.conditionVariable.notify()

    def avoid_callback(self, av):
        self.avoid_direction = av.data

        if av.data > 0:
            self.is_manual = False

    def rcout_callback(self, rcoutp):
        with self.nodeLock:
            self.last_rc3_raw = rcoutp.channels[2]

            if not self.is_manual:
                self.last_throttle = 0

    def state_callback(self, sp):
        with self.nodeLock:
            self.fcu_mode = sp.mode


    def constrain(self, v, nmin, nmax):
        return max(min(nmax, v), nmin)

    def check_end(self):
        if self.is_manual:
            if self.manual_start_time == None:
                self.manual_start_time = datetime.now()
            elif self.avoid_direction != 3:
                diff = datetime.now() - self.manual_start_time
                if diff.total_seconds() > 60 and not self.is_debug:
                    rospy.loginfo("%s: EndManualThread in manual for %f, stopping search and go back to AUTO" % (self.node_name, diff.total_seconds()))
                    self.set_auto()
        else:
            self.manual_start_time = None

        return

    def set_auto(self):
        rospy.loginfo("%s: continue AUTO" % self.node_name)

        if not self.is_manual:
            rospy.loginfo("%s: already set to AUTO" % self.node_name)
            return

        try:
            self.search_pub.publish(False)
        except:
            rospy.logerr("search_pub() exception")

        try:
            self.sound_pub.publish(2)
        except:
            rospy.logerr("sound_pub() exception")

        mode = None
        with self.nodeLock:
            mode = self.fcu_mode

        if mode == None or mode != 'AUTO':
            orc = OverrideRCIn();
            orc.channels[0] = 0
            orc.channels[1] = 0
            orc.channels[2] = 0
            orc.channels[3] = 0
            orc.channels[4] = 0
            orc.channels[5] = 0
            orc.channels[6] = 0
            orc.channels[7] = 0

            try:
                self.rc_override_pub.publish(orc)
            except:
                rospy.logerr("rc_override_pub() exception")

            ret = None
            try:
                ret = self.set_mode(base_mode = 0, custom_mode = 'AUTO')
            except rospy.ServiceException as ex:
                rospy.logerr(ex)

            if not ret.success:
                rospy.logerr("set_mode(0, AUTO) failed. Check mavros logs")

            time.sleep(0.2)

        self.is_manual = False

