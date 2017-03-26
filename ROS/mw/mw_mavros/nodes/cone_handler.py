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
from mavros_msgs.msg import OverrideRCIn, ParamValue, Waypoint
from mavros_msgs.srv import ParamGet, ParamSet, SetMode
from mw_video.msg import ConeLocation
from geometry_msgs.msg import TwistStamped

nodeLock = threading.Lock()

class ConeHandler():
    def __init__(self):
        self.node_name = "mwConeHandler"
        self.is_manual = False
        self.touch_value = 0
        self.arduino_speed_value = 0.0
        self.manual_start_time = None
        self.avoid_direction = 0                    # 1 - Left, 2 - Right
        self.z_angular_velocity = 0.0
        self.x_linear_velocity = 0.0
        self.last_throttle = 50
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
        self.apm_rc1_rev = 0
        self.apm_rc1_trim = 0

        self.apm_rc3_trim = 0

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
        self.gp_vel_sub = rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, self.gp_vel_callback, queue_size = 1)

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
            ret = self.get_param(param_id = 'RC1_REV')
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and ret.success:
            self.apm_rc1_rev = ret.value.integer
        else:
            rospy.logerr("get_param(RC1_REV) request failed. Check mavros logs")

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
        rospy.loginfo("   _tconst= %f  _p= %f  _i= %f  _d= %f  _imax= %d  _minspd= %f" %
            (self.apm_steer2srv_tconst, self.apm_steer2srv_p, self.apm_steer2srv_i,
             self.apm_steer2srv_d, self.apm_steer2srv_imax, self.apm_steer2srv_minspd))
        rospy.loginfo("   rc1 _max= %d  _min= %d  _rev= %d  _trim= %d" %
            (self.apm_rc1_max, self.apm_rc1_min, self.apm_rc1_rev, self.apm_rc1_trim))
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

    def gp_vel_callback(self, ts):
        nodeLock.acquire()
        try:
            self.x_linear_velocity = ts.twist.linear.x
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
        if self.apm_rc1_rev == -1:
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
        last_t = self.last_throttle
        target_vel = 0.5
        curr_vel = self.arduino_speed_value

        if distance < 3:
            # return 50 # 75 # 20150403 84 # 77
            # return 65 # 78 # 20150403 86 # 83
            # return 75 # 85 # 20150403 95
            target_vel = 0.5
            if curr_vel < target_vel * 0.95:
                if self.last_throttle < 110:
                    self.last_throttle = self.last_throttle + 2
            elif curr_vel > target_vel * 1.05:
                if self.last_throttle > 50:
                    self.last_throttle = self.last_throttle - 2
        elif distance < 5:
            target_vel = 0.75
            if curr_vel < target_vel * 0.95:
                if self.last_throttle < 110:
                    self.last_throttle = self.last_throttle + 5
            elif curr_vel > target_vel * 1.05:
                if self.last_throttle > 50:
                    self.last_throttle = self.last_throttle -5
        else:
            target_vel = 0.75
            if curr_vel < target_vel * 0.95:
                if self.last_throttle < 120:
                    self.last_throttle = self.last_throttle + 8
            elif curr_vel > target_vel * 1.05:
                if self.last_throttle > 50:
                    self.last_throttle = self.last_throttle -5

        rospy.loginfo("mw target_vel=%f curr_vel= %f  last_t= %d curr_t= %d" % (target_vel, curr_vel, last_t, self.last_throttle))

        return self.last_throttle

def main(args):       
    try:
        c = ConeHandler()

    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
