#!/usr/bin/env python

from datetime import datetime
import sys
import time
import rospy
import threading
from std_msgs.msg import Int8, Int16
from mavros_msgs.msg import OverrideRCIn, ParamValue

from mw_mavros.base_handler import BaseHandler

class TouchHandler(BaseHandler):
    states = {
        'neutral_1' : {
            'max_time' : 0.05, 
            'next' : 'brake'
        },
        'brake' : {
            'max_time' : 2.0,
            'target_speed' : 0.0,
            'target_time' : 0.1,
            'throttle_offset' : -150,
            'next' : 'neutral_2'
        },
        'neutral_2' : {
            'check_finish' : True,
            'max_time' : 0.05, 
            'next' : 'back'
        },
        'back' : {
            'max_time' : 10.0,
            'target_speed' : -1.0,
            'target_time' : 1.5,
            'reset_pid' : True,
            'throttle_max' : -0.1,
            'throttle_min' : -600.0,
            'next' : 'neutral_3'
        },
        'neutral_3' : {
            'max_time' : 0.5, 
            'next' : 'turn'
        },
        'turn' : {
            'max_time' : 10.0,
            'target_speed' : 1.0,
            'target_time' : 1.0,
            'reset_pid' : True,
            'steer_offset' : 200,
            'throttle_max' : 600.0,
            'throttle_min' : 0.1,
            'next' : 'forward'
        },
        'forward' : {
            'max_time' : 10.0,
            'target_speed' : 1.0,
            'target_time' : 0.5,
            'throttle_max' : 600.0,
            'throttle_min' : 0.1,
            'next' : 'neutral_4'
        },
        'neutral_4' : {
            'max_time' : 0.1, 
            'next' : 'done'
        },
        'done' : {
        }
    }

            
    def __init__(self):
        self.node_name = "mwMavrosTouchHandler"

        super(self.__class__, self).__init__()

        self.current_wp = -1
        self.touch_value = -1
        self.state = None
        self.state_start = 0
        self.state_target_speed_reached = None

        self.current_sub = rospy.Subscriber("/mavros/mission/current", Int16, self.current_callback, queue_size = 1)
        self.touch_sub = rospy.Subscriber("/mw/touch", Int8, self.touch_callback, queue_size = 1)

        while not rospy.is_shutdown():
            self.check_end()
            self.send_rc_command()

            time.sleep(0.02)

        rospy.loginfo("%s: Done." % self.node_name)

    def current_callback(self, cp):
        self.current_wp = cp.data

        if self.state == 'done' and self.current_wp == 1:
            self.state = None

    def touch_callback(self, tp):
        self.nodeLock.acquire()
        try:
            touch = tp.data
        finally:
            self.nodeLock.release()

        if self.touch_value == touch:
            return

        self.touch_value = touch
        
        if touch == 0:
            return

        if not self.is_manual:
            rospy.loginfo("%s: touch_callback %d - Set MANUAL" % (self.node_name, self.touch_value))

            ret = None
            try:
                ret = self.set_mode(base_mode = 0, custom_mode = 'MANUAL')
            except rospy.ServiceException as ex:
                rospy.logerr(ex)

            if ret != None and ret.success:
                self.is_manual = True
            else:
                rospy.logerr("Request failed. Check mavros logs")

    def advance_state(self):
        if 'check_finish' in self.states[self.state].keys():
            if self.avoid_direction == 3:                          # Finish/Stop
                self.state = 'done'
                orc = OverrideRCIn()
                orc.channels[0] = 0
                orc.channels[1] = 0
                orc.channels[2] = 0
                orc.channels[3] = 0
                orc.channels[4] = 0
                orc.channels[5] = 0
                orc.channels[6] = 0
                orc.channels[7] = 0
                rospy.loginfo("Send rc: %d %d" % (orc.channels[0], orc.channels[2]))
                self.rc_override_pub.publish(orc)                  # Release
                self.sound_pub.publish(0)
                self.is_manual = False

                return True                                            # XXX: For now!

        self.state = self.states[self.state]['next']
        self.state_start = datetime.now()
        self.state_target_speed_reached = None

        if self.state == 'done':
            if self.current_wp > 1:
                rospy.loginfo("%s: continue AUTO current wp= %d" % (self.node_name, self.current_wp))
                self.set_auto()

            return True

        if 'reset_pid' in self.states[self.state].keys():
            self.throttle_pid.reset_i()
            self.last_throttle = 0

        rospy.loginfo("%s: new state: %s" % (self.node_name, self.state))

        return False

    def send_rc_command(self):
        if not self.is_manual:
            return

        if self.state == 'done':
            return

        self.nodeLock.acquire()
        try:
            avoid = self.avoid_direction
            speed = self.arduino_speed_value
            touch = self.touch_value
            last_rc3 = self.last_rc3_raw
        finally:
            self.nodeLock.release()

        if self.state == None:
            self.state = 'neutral_1'
            self.state_start = datetime.now()
            self.throttle_pid.reset_i()
            self.last_throttle = 0

        diff = (datetime.now() - self.state_start).total_seconds()
        if diff >= self.states[self.state]['max_time']:
            if self.advance_state():
                return

        if 'target_speed' in self.states[self.state].keys():
            if abs(speed - self.states[self.state]['target_speed']) <= 0.2 and self.state_target_speed_reached == None:
                self.state_target_speed_reached = datetime.now()

            if self.state_target_speed_reached != None:
                diff = (datetime.now() - self.state_target_speed_reached).total_seconds()
                if diff >= self.states[self.state]['target_time']:
                    if self.advance_state():
                        return

        orc = OverrideRCIn()
        orc.channels[1] = 0
        orc.channels[3] = 0
        orc.channels[4] = 0
        orc.channels[5] = 0
        orc.channels[6] = 0
        orc.channels[7] = 0

        orc.channels[0] = self.apm_rc1_trim                    #  straight

        if 'steer_offset' in self.states[self.state].keys():
            if avoid > 0:
                if avoid == 1:
                    orc.channels[0] -= self.states[self.state]['steer_offset']
                else:
                    orc.channels[0] += self.states[self.state]['steer_offset']
            else:
                if touch < 2:
                    orc.channels[0] += self.states[self.state]['steer_offset']
                else:
                    orc.channels[0] -= self.states[self.state]['steer_offset']

        orc.channels[2] = self.apm_rc3_trim                    # neutral

        if 'target_speed' in self.states[self.state].keys():
            if 'throttle_offset' in self.states[self.state].keys():
                orc.channels[2] += self.states[self.state]['throttle_offset']
            else:
                if self.last_throttle == 0:
                    if last_rc3 - self.apm_servo3_trim < 0:
                        self.last_throttle = float(last_rc3 - self.apm_servo3_trim) / float(self.apm_servo3_trim - self.apm_servo3_min) * 1000.0
                    else:
                        self.last_throttle = float(last_rc3 - self.apm_servo3_trim) / float(self.apm_servo3_max - self.apm_servo3_trim) * 1000.0

                last_t = self.last_throttle
                target_speed = self.states[self.state]['target_speed']
                error = target_speed - speed

                last_t += self.throttle_pid.get_pid(error, 7.5)
                last_t = self.constrain(last_t, self.states[self.state]['throttle_min'], self.states[self.state]['throttle_max']) # 0.1 of percents

                if last_t < 0.0:
                    orc.channels[2] += int(last_t / 1000.0 * float(self.apm_servo3_trim - self.apm_servo3_min))
                else:
                    orc.channels[2] += int(last_t / 1000.0 * float(self.apm_servo3_max - self.apm_servo3_trim))

                self.last_throttle = last_t

        rospy.loginfo("Send rc: %d %d" % (orc.channels[0], orc.channels[2]))
        self.rc_override_pub.publish(orc)

def main(args):
    try:
        TouchHandler()

    except KeyboardInterrupt:
        print "close"

if __name__ == '__main__':
    main(sys.argv)
    
