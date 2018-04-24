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

from mw_mavros.base_handler import BaseHandler

class ConeHandler(BaseHandler):
    def __init__(self):
        self.node_name = "mwConeHandler"

        super(self.__class__, self).__init__()

        self.z_angular_velocity = 0.0
        self.cone_seen_time = datetime(1990, 1, 1)
        self.last_cone_seen_time = datetime(1990, 1, 1)
        self.last_touch = datetime(1990, 1, 1)
        self.last_time = datetime.now()
        self.release_sent = False
        self.last_out = 0.0
        self.direction = 0
        self.integrator = 0.0

        self.apm_steer2srv_tconst = 0.75
        self.apm_steer2srv_p = 2.05
        self.apm_steer2srv_i = 0.2
        self.apm_steer2srv_d = 0.005
        self.apm_steer2srv_imax = 1500
        self.apm_steer2srv_minspd = 1.0

        self.cone_sub = rospy.Subscriber("/mw/cone_location", ConeLocation, self.cone_callback, queue_size = 1)
        self.touch_sub = rospy.Subscriber("/mw/touch", Int8, self.touch_callback, queue_size = 1)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback, queue_size = 1)

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'STEER2SRV_TCONST')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     self.apm_steer2srv_tconst = ret.value.real
        # else:
        #     rospy.logerr("get_param(STEER2SRV_TCONST) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_STR_RAT_P')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     self.apm_steer2srv_p = ret.value.real
        # else:
        #     rospy.logerr("get_param(ATC_STR_RAT_P) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_STR_RAT_I')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     self.apm_steer2srv_i = ret.value.real
        # else:
        #     rospy.logerr("get_param(ATC_STR_RAT_I) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_STR_RAT_D')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     self.apm_steer2srv_d = ret.value.real
        # else:
        #     rospy.logerr("get_param(ATC_STR_RAT_D) request failed. Check mavros logs")

        # ret = None
        # try:
        #     ret = self.get_param(param_id = 'ATC_STR_RAT_IMAX')
        # except rospy.ServiceException as ex:
        #     rospy.logerr(ex)

        # if ret != None and ret.success:
        #     self.apm_steer2srv_imax = ret.value.integer
        # else:
        #     rospy.logerr("get_param(ATC_STR_RAT_IMAX) request failed. Check mavros logs")

        rospy.loginfo("   _tconst= %f  _p= %f  _i= %f  _d= %f  _imax= %d  _minspd= %f" %
            (self.apm_steer2srv_tconst, self.apm_steer2srv_p, self.apm_steer2srv_i,
             self.apm_steer2srv_d, self.apm_steer2srv_imax, self.apm_steer2srv_minspd))

        while not rospy.is_shutdown():
            self.check_end()
            self.send_rc_command()

            with self.conditionVariable:
                if self.updates > 0:
                    self.updates = 0
                else:
                    self.conditionVariable.wait(0.02)

        rospy.loginfo("%s: Done." % self.node_name)

    def imu_callback(self, imup):
        with self.nodeLock:
            self.z_angular_velocity = math.degrees(imup.angular_velocity.z)

    def touch_callback(self, tp):
        self.touch_value = tp.data
        if self.touch_value > 0:
            self.is_manual = False
            self.last_touch = datetime.now()

            try:
                self.search_pub.publish(False)
            except:
                rospy.logerr("search_pub() exception")

            try:
                self.sound_pub.publish(2)
            except:
                rospy.logerr("sound_pub() exception")

    def cone_callback(self, cp):
        with self.nodeLock:
            self.distance = cp.distance
            self.delta_heading = cp.heading
            self.cone_seen_time = datetime.now()

        rospy.loginfo("distance: %f  delta_heading: %f" % (self.distance, self.delta_heading))

        if not self.is_manual and self.touch_value == 0 and (self.cone_seen_time - self.last_touch).total_seconds() > 1.0:
            # 1 second to make sure, all the old/parallel image messages are handled 
            rospy.loginfo("%s: cone_callback - Set MANUAL" % (self.node_name))

            ret = None
            try:
                ret = self.set_mode(base_mode = 0, custom_mode = 'MANUAL')
            except rospy.ServiceException as ex:
                rospy.logerr(ex)

            if ret != None and ret.success:
                self.is_manual = True
                self.direction = 0
            else:
                rospy.logerr("Request failed. Check mavros logs")

        with self.conditionVariable:
            self.updates += 1
            self.conditionVariable.notify()

    def send_rc_command(self):
        if self.is_manual:
            with self.nodeLock:
                z_angular_velocity = self.z_angular_velocity
                distance = self.distance
                delta_heading = self.delta_heading

                tdiff = self.cone_seen_time - self.last_cone_seen_time
                if tdiff.total_seconds() != 0:
                    self.last_cone_seen_time = self.cone_seen_time
                    self.release_sent = False

                self.updates = 0

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
        if self.apm_rc1_reversed != 0:
            reverse_mul = -1

        if angle * reverse_mul > 0:
            return int(reverse_mul * (angle * (self.apm_rc1_max - self.apm_rc1_trim)) / _high)
        else:
            return int(reverse_mul * (angle * (self.apm_rc1_trim - self.apm_rc1_min)) / _high)

    # Stolen & adapted from ardupilot/libraries/APM_Control/AP_SteerController.cpp
    def calc_steering(self, desired_rate, z_angular_velocity):
        # z_angular_velocity comes in negated:-(
        z_angular_velocity = 0.0 - z_angular_velocity

        tnow = datetime.now()

        tdiff = tnow - self.last_time
        dt = tdiff.total_seconds() * 1000.0

        if dt > 1000.0:
            dt = 0.0

        self.last_time = tnow

        speed = self.arduino_speed_value

        if speed < self.apm_steer2srv_minspd:
            # assume a minimum speed. This stops oscillations when first starting to move
            speed = self.apm_steer2srv_minspd

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
            rospy.loginfo("mw last_rc3_raw= %d  apm_rc3_trim= %d  apm_rc3_max= %d" % (self.last_rc3_raw, self.apm_rc3_trim, self.apm_rc3_max))

            if self.last_rc3_raw - self.apm_rc3_trim < 0:
                # should not be negative
                self.last_throttle = 0.01
            else:
                self.last_throttle = float(self.last_rc3_raw - self.apm_rc3_trim) / float(self.apm_rc3_max - self.apm_rc3_trim) * 1000.0

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

        rospy.loginfo("calc_throttle speed= %f  error= %f  direction= %f " % (curr_vel, error, self.direction))

        if self.direction <= 0 and error <= -1.0:
            # Brake for a bit
            neg_now = (datetime(2000, 1, 1) - datetime.now()).total_seconds()

            if self.direction == 0:
                # short neutral
                self.direction = neg_now
                rospy.loginfo("mw calc_throttle neutral")

                return 0
            elif abs(neg_now - self.direction)  < 0.05:
                rospy.loginfo("mw calc_throttle neutral wait")
                return 0
            elif abs(neg_now - self.direction) < 1.5:
                # limit the brake time`
                self.last_throttle = 0.01
                rospy.loginfo("mw brake = %d" % int(float(self.apm_rc3_min - self.apm_rc3_trim) * self.apm_braking_percent / 100.0))

                return int(float(self.apm_rc3_min - self.apm_rc3_trim) * self.apm_braking_percent / 100.0)
            else:
                self.direction = (datetime.now() - datetime(2000, 1, 1)).total_seconds()

        if self.direction > 1:
            pos_now = (datetime.now() - datetime(2000, 1, 1)).total_seconds()
            if pos_now - self.direction < 0.05:
                rospy.loginfo("mw calc_throttle neutral 2 wait")
                # short neutral
                return 0

        rospy.loginfo("mw forward")

        self.direction = 1

        self.last_throttle += self.throttle_pid.get_pid(error, 7.5)
        self.last_throttle = self.constrain(self.last_throttle, 0.01, 1000.0)

        thr_rc = int(self.last_throttle / 1000.0 * float(self.apm_rc3_max - self.apm_rc3_trim))

        rospy.loginfo("mw target_vel=%f  curr_vel= %f  last_t= %f  curr_t= %f thr_rc= %d" % (target_vel, curr_vel, last_t, self.last_throttle, thr_rc))

        return thr_rc

def main(args):       
    try:
        c = ConeHandler()

    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
