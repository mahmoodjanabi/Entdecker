#!/usr/bin/env python

import rospy
import serial
import sys
from std_msgs.msg import Int8
from mavros.srv import CommandLong

class SoundHandler():
    def __init__(self):
        self.node_name = "mwMavrosSoundHandler"

        rospy.init_node(self.node_name)

        rospy.wait_for_service('/mavros/cmd/command')
        self.set_command = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        self.sound_sub = rospy.Subscriber("/mw/sound", Int8, self.sound_callback)
        
        rospy.loginfo("%s: Ready." % self.node_name)

    def sound_callback(self, sp):
        sound = sp.data
        r0_value = 0
        r1_value = 0

        rospy.loginfo("%s.sound_callback(%d)" % (self.node_name, sound))

        if sound == 1:
            r0_value = 1
        elif sound == 2:
            r1_value = 1
        
        ret = None
        try:
            ret = self.set_command(command = 181,                  # CommandLong.CMD_DO_SET_RELAY
                    param1 = 0, param2 = r0_value)
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and not ret.success:
            rospy.logerr("set_command() request failed. Check mavros logs")

        ret = None
        try:
            ret = self.set_command(command = 181,                  # CommandLong.CMD_DO_SET_RELAY
                    param1 = 1, param2 = r1_value)
        except rospy.ServiceException as ex:
            rospy.logerr(ex)

        if ret != None and not ret.success:
            rospy.logerr("set_command() request failed. Check mavros logs")

def main(args):
    try:
        SoundHandler()
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("close")

if __name__ == '__main__':
    main(sys.argv)
    
