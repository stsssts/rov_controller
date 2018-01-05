#!/usr/bin/env python

import math
import rospy

from bridge import ArduinoBoard
from thrusters_controller import ThrustersController
from openrov_msgs.msg import RovMotion


class RovController:
    def __init__(self):
        self.arduino = ArduinoBoard(self.parse_message)
        self.thrust_controller = ThrustersController()
        self.max_level = 100.0

        if rospy.has_param('thrusters'):
            self.thrust_controller.configure(rospy.get_param('thrusters'))

        rospy.Subscriber("/rov/move", RovMotion, self.move_callback)

    def move_callback(self, msg):
        if msg.max_level and msg.max_level != self.max_level:
            self.max_level = msg.max_level
        throttle = int(msg.throttle * self.max_level / 100.)
        lift = int(msg.lift * self.max_level / 100.)
        yaw = int(msg.yaw * self.max_level / 100.)

        p, v, s = self.thrust_controller.get_pwm2x1(throttle, lift, yaw)
        self.arduino.send('go(%s,%s,%s,0);' % (p, v, s))

    def parse_message(self, msg):
    	#if 'vout' in msg:
        #    rospy.loginfo(msg)
        if 'crc:' in msg:
            rospy.loginfo(msg)
        if 'pong:' in msg:
            rospy.loginfo(msg)

    def construct_message(self, command, value=''):
        return command + '(' + str(value) + ');'

    def ping(self, event):
        self.arduino.send(self.construct_message('ping', 0))



if __name__ == '__main__':
    rospy.init_node('bottom_side', anonymous=True)
    core = RovController() #TODO

    rate = rospy.Rate(10) # 10hz
    rospy.Timer(rospy.Duration(1), core.ping)

    while not rospy.is_shutdown():
        rate.sleep()
