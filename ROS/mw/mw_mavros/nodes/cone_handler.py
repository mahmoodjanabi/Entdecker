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
from mavros_msgs.msg import OverrideRCIn, ParamValue, RCOut, Waypoint
from mavros_msgs.srv import ParamGet, ParamSet, SetMode
from mw_video.msg import ConeLocation
from geometry_msgs.msg import TwistStamped

nodeLock = threading.Lock()

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

class ConeHandler():
    def __init__(self):
        self.node_name = "mwConeHandler"
        self.is_manual = False
        self.touch_value = 0
        self.arduino_speed_value = 0.0
        self.manual_start_time = None
        self.avoid_direction = 0                    # 1 - Left, 2 - Right
        self.z_angular_velocity = 0.0
        self.last_throttle = 0
        self.cone_seen_time = datetime(1990, 1, 1)
        self.last_cone_seen_time = datetime(1990, 1, 1)
        self.last_time = datetime.now()
        self.release_sent = False
        self.last_out = 0.0
        self.integrator = 0.0

        self.apm_steer2srv_tconst = 0.75
        self.apm_steer2srv_p = 1.8
        self.apm_steer2srv_i = 0.2
        self.apm_steer2srv_d = 0.005
        self.apm_steer2srv_imax = 1500
        self.apm_steer2srv_minspd = 1.0

        self.apm_rc1_max = 0
        self.apm_rc1_min = 0
        self.apm_rc1_reversed = 0
        self.apm_rc1_trim = 0

        self.apm_rc3_trim = 0
        self.apm_servo3_max = 0
        self.apm_servo3_min = 0
        self.apm_servo3_trim = 0
        self.last_auto_rc3_raw = 0

        self.throttle_pid = None

        self.auto_kickstart = -1

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
        self.cone_sub = rospy.Subscriber("/mw/cone_location", ConeLocation, self.cone_callback, queue_size = 1)
        self.touch_sub = rospy.Subscriber("/mw/touch", Int8, self.touch_callback, queue_size = 1)
        self.arduino_speed_sub = rospy.Subscriber("/mw/speed", Float32, self.arduino_speed_callback, queue_size = 1)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback, queue_size = 1)
        self.rcout_sub = rospy.Subscriber("/mavros/rc/out", RCOut, self.rcout_callback, queue_size = 1)

        ret = None
        try:
            ret = self.get_param(param_id = 'STEER2SRV_TCONST')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_steer2srv_tconst = ret.value.real
        else:
            rospy.logerr("get_param(STEER2SRV_TCONST) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'STEER2SRV_P')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_steer2srv_p = ret.value.real
        else:
            rospy.logerr("get_param(STEER2SRV_P) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'STEER2SRV_I')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_steer2srv_i = ret.value.real
        else:
            rospy.logerr("get_param(STEER2SRV_I) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'STEER2SRV_D')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_steer2srv_d = ret.value.real
        else:
            rospy.logerr("get_param(STEER2SRV_D) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'STEER2SRV_IMAX')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_steer2srv_imax = ret.value.integer
        else:
            rospy.logerr("get_param(STEER2SRV_IMAX) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'STEER2SRV_MINSPD')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_steer2srv_minspd = ret.value.real
        else:
            rospy.logerr("get_param(STEER2SRV_MINSPD) request failed. Check mavros logs")

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

        ret = None
        try:
            ret = self.get_param(param_id = 'SERVO3_MAX')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_servo3_max = ret.value.integer
        else:
            rospy.logerr("get_param(SERVO3_MAX) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'SERVO3_MIN')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_servo3_min = ret.value.integer
        else:
            rospy.logerr("get_param(SERVO3_MIN) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'SERVO3_TRIM')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_servo3_trim = ret.value.integer
        else:
            rospy.logerr("get_param(SERVO3_TRIM) request failed. Check mavros logs")

        thr_p = 0.0
        thr_i = 0.0
        thr_d = 0.0
        thr_imax = 0.0

        ret = None
        try:
            ret = self.get_param(param_id = 'SPEED2THR_P')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            thr_p = ret.value.real
        else:
            rospy.logerr("get_param(SPEED2THR_P) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'SPEED2THR_I')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            thr_i = ret.value.real
        else:
            rospy.logerr("get_param(SPEED2THR_I) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'SPEED2THR_D')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            thr_d = ret.value.real
        else:
            rospy.logerr("get_param(SPEED2THR_D) request failed. Check mavros logs")

        ret = None
        try:
            ret = self.get_param(param_id = 'SPEED2THR_IMAX')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            thr_imax = ret.value.integer
        else:
            rospy.logerr("get_param(SPEED2THR_IMAX) request failed. Check mavros logs")

        self.throttle_pid = PID(thr_p, thr_i, thr_d)
        self.throttle_pid._imax = thr_imax

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
        rospy.loginfo("   _tconst= %f  _p= %f  _i= %f  _d= %f  _imax= %d  _minspd= %f" %
            (self.apm_steer2srv_tconst, self.apm_steer2srv_p, self.apm_steer2srv_i,
             self.apm_steer2srv_d, self.apm_steer2srv_imax, self.apm_steer2srv_minspd))
        rospy.loginfo("   rc1 _max= %d  _min= %d  _rev= %d  _trim= %d" %
            (self.apm_rc1_max, self.apm_rc1_min, self.apm_rc1_reversed, self.apm_rc1_trim))
        rospy.loginfo("   rc3 _trim= %d" %
            (self.apm_rc3_trim))

        while not rospy.is_shutdown():
            self.check_end()
            self.send_rc_command()

            time.sleep(0.02)

        rospy.loginfo("%s: Done." % self.node_name)

    def avoid_callback(self, av):
        self.avoid_direction = av.data

        if av.data > 0:
            self.is_manual = False

    def imu_callback(self, imup):
        nodeLock.acquire()
        try:
            self.z_angular_velocity = math.degrees(imup.angular_velocity.z)
        finally:
            nodeLock.release()

    def rcout_callback(self, rcoutp):
        if not self.is_manual:
            nodeLock.acquire()
            try:
                self.last_auto_rc3_raw = rcoutp.channels[2]
                self.last_throttle = 0
            finally:
                nodeLock.release()

    def touch_callback(self, tp):
        self.touch_value = tp.data
        if self.touch_value > 0:
            self.is_manual = False
        # rospy.loginfo("touch_callback value= %d is_manual= %d" % (self.touch_value, self.is_manual))

    def arduino_speed_callback(self, tp):
        self.arduino_speed_value = tp.data

    def cone_callback(self, cp):
        nodeLock.acquire()
        try:
            self.distance = cp.distance
            self.delta_heading = cp.heading
            self.cone_seen_time = datetime.now()
        finally:
            nodeLock.release()

        rospy.loginfo("distance: %f  delta_heading: %f" % (self.distance, self.delta_heading))

        if not self.is_manual and self.touch_value == 0:
            rospy.loginfo("%s: cone_callback - Set MANUAL" % (self.node_name))

            ret = None
            try:
                ret = self.set_mode(base_mode = 0, custom_mode = 'MANUAL')
            except rospy.ServiceException as ex:
                rospy.logerr(ex)

            if ret != None and ret.success:
                self.is_manual = True
            else:
                rospy.logerr("Request failed. Check mavros logs")

    def set_auto(self):
        rospy.loginfo("%s: continue AUTO" % self.node_name)

        if not self.is_manual:
            rospy.loginfo("%s: already set to AUTO" % self.node_name)
            return

        retry = 2
        while retry > 0:
            try:
                self.search_pub.publish(False)
            except:
                rospy.logerr("search_pub() exception")
                retry = retry - 1
            else:
                retry = 0

        retry = 2
        while retry > 0:
            try:
                self.sound_pub.publish(2)
            except:
                rospy.logerr("sound_pub() exception")
                retry = retry - 1
            else:
                retry = 0

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

        time.sleep(0.2)

        ret = None
        try:
            ret = self.set_param(param_id = 'AUTO_KICKSTART',
                value = ParamValue(integer = 0, real = self.auto_kickstart))
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret == None or not ret.success:
            rospy.logerr("set_param(AUTO_KICKSTART, %f) request failed. Check mavros logs" % self.auto_kickstart)

        self.is_manual = False

    def check_end(self):
        if self.is_manual:
            if self.manual_start_time == None:
                self.manual_start_time = datetime.now()
            elif self.avoid_direction != 3:
                diff = datetime.now() - self.manual_start_time
                if diff.total_seconds() > 60 and not self.is_debug:
                    rospy.loginfo("EndManualThread in manual for %f, stopping search and go back to AUTO" % diff.total_seconds())
                    self.set_auto()
        else:
            self.manual_start_time = None

        return

    def send_rc_command(self):
        if self.is_manual:
            nodeLock.acquire()
            try:
                z_angular_velocity = self.z_angular_velocity
                distance = self.distance
                delta_heading = self.delta_heading

                tdiff = self.cone_seen_time - self.last_cone_seen_time
                if tdiff.total_seconds() != 0:
                    self.last_cone_seen_time = self.cone_seen_time
                    self.release_sent = False
            finally:
                nodeLock.release()

            rospy.loginfo("rc thread z_ang= %f, dist= %f, delta_hdg= %f" % (z_angular_velocity, distance, delta_heading))

            if not self.release_sent:
                tdiff = datetime.now() - self.last_cone_seen_time 
                if tdiff.total_seconds() > 0.75 and not self.is_debug:
                    rospy.loginfo("rc thread  datetime.now() - self.last_cone_seen_time = %f >  0.75", tdiff.total_seconds())
                    self.set_auto()
                    self.release_sent = True
                else:
                    orc = OverrideRCIn();
                    orc.channels[0] = self.apm_rc1_trim + self.calc_steering(delta_heading, z_angular_velocity)
                    orc.channels[1] = 0
                    orc.channels[2] = self.apm_rc3_trim + self.calc_throttle(distance)
                    orc.channels[3] = 0
                    orc.channels[4] = 0
                    orc.channels[5] = 0
                    orc.channels[6] = 0
                    orc.channels[7] = 0

                    rospy.loginfo("Send rc: %d %d" % (orc.channels[0], orc.channels[2]))
                    self.rc_override_pub.publish(orc)
        else:
            # Reinitalize state
            self.last_time = datetime.now()
            self.last_out = 0.0
            self.integrator = 0.0

        return

    def rc1_angle_to_pwm(self, angle):
        _high = 4500
        reverse_mul = 1
        if self.apm_rc1_reversed == -1:
            reverse_mul = -1

        if angle * reverse_mul > 0:
            return int(reverse_mul * (angle * (self.apm_rc1_max - self.apm_rc1_trim)) / _high)
        else:
            return int(reverse_mul * (angle * (self.apm_rc1_trim - self.apm_rc1_min)) / _high)

    def constrain(self, v, nmin, nmax):
        return max(min(nmax, v), nmin)

    # Stolen & adapted from ardupilot/libraries/APM_Control/AP_SteerController.cpp
    def calc_steering(self, desired_rate, z_angular_velocity):
        tnow = datetime.now()

        tdiff = tnow - self.last_time
        dt = tdiff.total_seconds() * 1000

        if dt > 1000:
            dt = 0

        self.last_time = tnow

        # XXX: find a better way for the speed
        speed = self.apm_steer2srv_minspd / 2

        # this is a linear approximation of the inverse steering
        # equation for a ground vehicle. It returns steering as an angle from -45 to 45
        scaler = 1.0 / speed

        # Calculate the steering rate error (deg/sec) and apply gain scaler
        rate_error = (desired_rate - z_angular_velocity) * scaler
        
        # Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
        # No conversion is required for K_D
        ki_rate = self.apm_steer2srv_i * self.apm_steer2srv_tconst * 45.0
        kp_ff = max((self.apm_steer2srv_p - self.apm_steer2srv_i * self.apm_steer2srv_tconst) *
            self.apm_steer2srv_tconst - self.apm_steer2srv_d , 0) * 45.0
        
        # Multiply roll rate error by _ki_rate and integrate
        # Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
        if ki_rate > 0 and speed >= self.apm_steer2srv_minspd:
            # only integrate if gain and time step are positive.
            if dt > 0:
                integrator_delta = rate_error * ki_rate * dt * 0.001 * scaler
                # prevent the integrator from increasing if steering defln demand is above the upper limit
                if self.last_out < -45:
                    integrator_delta = max(integrator_delta , 0)
                elif self.last_out > 45:
                    # prevent the integrator from decreasing if steering defln demand is below the lower limit
                    integrator_delta = min(integrator_delta, 0)

                self.integrator += integrator_delta;
        else:
            self.integrator = 0.0
        
        # Scale the integration limit
        intLimScaled = self.apm_steer2srv_imax * 0.01

        # Constrain the integrator state
        self.integrator = self.constrain(self.integrator, -intLimScaled, intLimScaled)
        
        # Calculate the demanded control surface deflection
        self.last_out = (rate_error * self.apm_steer2srv_d * 4.0) + (math.radians(desired_rate) * kp_ff) * scaler + self.integrator
        
        # Convert to centi-degrees and constrain
        ret = self.constrain(self.last_out * 100, -4500, 4500)

        # rospy.loginfo("calc_steering target deg= %d" % ret)
        # Translate into servo value
        return self.rc1_angle_to_pwm(ret)

    def calc_throttle(self, distance):
        if self.last_throttle == 0:
            # Find the last throttle used in auto mode (0.0 ... 1000.0)
            # Similar to my change APMrover which uses 0.1s of pecents
            rospy.loginfo("mw last_auto_rc3_raw= %d  apm_servo3_trim= %d  apm_servo3_max= %d" % (self.last_auto_rc3_raw, self.apm_servo3_trim, self.apm_servo3_max))

            if self.last_auto_rc3_raw - self.apm_servo3_trim < 0:
                # should not be negative
                self.last_throttle = 0.01
            else:
                self.last_throttle = float(self.last_auto_rc3_raw - self.apm_servo3_trim) / float(self.apm_servo3_max - self.apm_servo3_trim) * 1000.0

            self.throttle_pid.reset_i()

        last_t = self.last_throttle
        curr_vel = self.arduino_speed_value

        if distance < 3:
            target_vel = 0.5
        elif distance < 5:
            target_vel = 0.75
        else:
            target_vel = 1.0

        error = target_vel - curr_vel

        self.last_throttle += self.throttle_pid.get_pid(error, 5.0)
        self.last_throttle = self.constrain(self.last_throttle, 0.01, 1000.0)

        thr_rc = int(self.last_throttle / 1000.0 * float(self.apm_servo3_max - self.apm_servo3_trim))

        rospy.loginfo("mw target_vel=%f  curr_vel= %f  last_t= %f  curr_t= %f thr_rc= %d" % (target_vel, curr_vel, last_t, self.last_throttle, thr_rc))

        return thr_rc

def main(args):       
    try:
        c = ConeHandler()

    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
