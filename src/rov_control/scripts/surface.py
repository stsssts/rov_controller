#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from rov_control.msg import ControlData
from sensor_msgs.msg import Joy

class RovController:
    def __init__ (self):
        self.ROTATE_BUTTON = 0
	self.STRAFE_BUTTON = 3
	self.STOP_BUTTON = 1

	self.THRESHOLD_AXIS = 4
	self.DEPTH_AXIS = 2
	self.FORWARD_AXIS = 1
	self.TURN_AXIS = 0

        self.BLOCK_MODE = True

        rospy.init_node('bottom_side', anonymous=True)
        self.publisher = rospy.Publisher('rov_control', ControlData, queue_size=10)
        rospy.Subscriber("joy", Joy, self.callback)
        self.last_message = ControlData()
        self.last_message.stop = True
        self.received_message = False
        self.THRESHOLD = 0.25
        self.THRESHOLD_STEP = 0.01

        self.dirs = {}
        self.dirs["FL"] = 1.0
        self.dirs["FR"] = 1.0
        self.dirs["TL"] = 1.0
        self.dirs["TR"] = 1.0

    def rotate_moving(self, data, message):
        message.top_left = message.top_right = data.axes[self.DEPTH_AXIS]        
        message.forward_left  =  data.axes[self.TURN_AXIS]
        message.forward_right = -data.axes[self.TURN_AXIS]        
        
        return message
    
    def strafe_moving(self, data, message):
        message.top_left  =  data.axes[self.TURN_AXIS]
        message.top_right = -data.axes[self.TURN_AXIS]        
       
        return message

    def normal_moving(self, data, message):
        message.top_left = message.top_right = data.axes[self.DEPTH_AXIS]        
        message.forward_left  = data.axes[self.FORWARD_AXIS]
        message.forward_right = data.axes[self.FORWARD_AXIS]
        if data.axes[self.TURN_AXIS] > 0:
            message.forward_left  *= message.forward_left - data.axes[self.TURN_AXIS]*math.copysign(1,message.forward_left)
        else:
            message.forward_right  *= message.forward_right + data.axes[self.TURN_AXIS]*math.copysign(1,message.forward_right)
        return message

    def apply_dirs(self, message):
        message.forward_left  *= self.dirs["FL"]
        message.forward_right *= self.dirs["FR"]
        message.top_left      *= self.dirs["TR"]
        message.top_right     *= self.dirs["TL"]

    def callback(self, data):
        self.THRESHOLD += data.axes[self.THRESHOLD_AXIS]*self.THRESHOLD_STEP
        if self.THRESHOLD > 1.0:
            self.THRESHOLD = 1.0
        elif self.THRESHOLD < 0:
            self.THRESHOLD = 0.0
        message = ControlData()
        if data.buttons[self.STOP_BUTTON] == 1 and data.buttons[self.ROTATE_BUTTON] == 1 and data.buttons[self.STRAFE_BUTTON] == 1:
            self.BLOCK_MODE = False
        elif data.buttons[self.STOP_BUTTON] == 1:
            self.BLOCK_MODE = True        
        elif data.buttons[self.ROTATE_BUTTON] == 1:
            self.rotate_moving(data, message)
        elif data.buttons[self.STRAFE_BUTTON] == 1:
            self.strafe_moving(data, message)
        else:
            self.normal_moving(data, message)
   
        message.stop = self.BLOCK_MODE 
        message.max_level = self.THRESHOLD
        self.apply_dirs(message)
        self.last_message = message
        self.received_message = True
        #print "message"
    
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
