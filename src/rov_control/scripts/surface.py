#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from rov_control.msg import ControlData
from sensor_msgs.msg import Joy

class RovController:
    def __init__ (self):
        self.axes = {}
        self.axes["forward"] = 1
        self.axes["lag"] = 0
        self.axes["depth"] = 2
        self.axes["yaw"] = 3
        self.axes["threshold"] = 4

        self.buttons = {}
        self.buttons["rotate"] = 0
        self.buttons["stop"]   = 1
        self.buttons["pitch_shift_increase"]  = 4
        self.buttons["pitch_shift_decrease"]  = 5

        self.pitch_shift = 0
        self.yaw_shift  = 0

        self.BLOCK_MODE = True

        rospy.init_node('surface_side', anonymous=True)
        self.publisher = rospy.Publisher('rov_control', ControlData, queue_size=10)
        rospy.Subscriber("joy", Joy, self.callback)
        self.last_message = ControlData()
        self.last_message.stop = True
        self.received_message = False
        self.THRESHOLD = 0.25
        self.THRESHOLD_STEP = 0.01
                      
        self.dirs = ControlData()
        self.dirs.forward_left  = 1.0
        self.dirs.forward_right = 1.0
        self.dirs.lag_front     = 1.0
        self.dirs.lag_back      = 1.0
        self.dirs.top_front     = 1.0
        self.dirs.top_back      = 1.0

    def trunc_value(value, high_border = 1, low_border = -1):
        if value > high_border:
            value = high_border
        elif self.THRESHOLD < low_border:
            value = low_border

    def control_threshold(self, data, message):
        self.THRESHOLD += data.axes[self.axes["threshold"]]*self.THRESHOLD_STEP
        self.THRESHOLD = trunc_value(self.THRESHOLD, 1.0, 0)

        message.max_level = self.THRESHOLD

    def control_stop(self, data, message):
        if data.buttons[self.buttons["stop"]] == 1 and data.buttons[self.buttons["rotate"]] == 1:
            self.BLOCK_MODE = False
        elif data.buttons[self.buttons["stop"]] == 1:
            self.BLOCK_MODE = True        
        message.stop = self.BLOCK_MODE 

    def control_vertical(self, data, message):
        if data.buttons[self.buttons["pitch_shift_increase"]] == 1:
            self.pitch_shift += 0.01
        elif data.buttons[self.buttons["pitch_shift_decrease"]]:
            self.pitch_shift -= 0.01
        self.pitch_shift = trunc_value(self.pitch_shift)

        message.top_front = message.top_back = data.axes[self.axes["depth"]]        
        message.top_front = trunc_value(message.top_front + self.pitch_shift)
        message.top_back  = trunc_value(message.top_back  - self.pitch_shift)

    def control_horisontal(self, data, message):
        yaw_shift = data.axes[self.axes["yaw"]]

        message.forward_left = message.forward_right = data.axes[self.axes["forward"]]
        message.lag_front    = message.lag_back      = data.axes[self.axes["lag"]] 

        message.forward_left  = trunc_value(yaw_shift + message.forward_left)
        message.forward_right = trunc_value(yaw_shift - message.forward_right)
        message.lag_front     = trunc_value(yaw_shift + message.lag_front)
        message.lag_back      = trunc_value(yaw_shift - message.lag_back)    

    def apply_dirs(self, message):
        message.forward_left  *= self.dirs.forward_left
        message.forward_right *= self.dirs.forward_right
        message.lag_front     *= self.dirs.lag_front
        message.lag_back      *= self.dirs.lag_back
        message.top_front     *= self.dirs.top_front
        message.top_back      *= self.dirs.top_back

    def callback(self, data):
        message = ControlData()

        self.control_stop(data, message)
        self.control_threshold(data, message)
        self.control_vertical(data, message)
        self.control_horisontal(data, message)
        
        self.apply_dirs(message)
        self.last_message = message
        self.received_message = True
    
    def loop(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            if self.received_message:
                rospy.loginfo(self.last_message)
                self.publisher.publish(self.last_message)
                self.received_message = False
            rate.sleep()

if __name__ == '__main__':
    core = RovController()
    core.loop()
