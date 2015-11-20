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

    def callback(self, data):
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
