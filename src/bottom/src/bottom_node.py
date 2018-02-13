#!/usr/bin/env python
from math import copysign
from time import sleep

import Adafruit_BBIO.PWM as PWM

import rospy
from rov_control.msg import ControlData

class MotorController():
  def __init__(self, port):
    self.MAX = 0.25
    self.stop_time  = 0.0015
    self.time_delta = 0.0004
    self.port = port
    self.frequency = 100.0
    PWM.start(self.port, 0, self.frequency)

  def set_power(self, power):
    if abs(power) > self.MAX:
      power = copysign(self.MAX, power)
    time = self.stop_time + power*self.time_delta
    percent = 100.0*time*self.frequency
    PWM.set_duty_cycle(self.port, percent)

class RovController:
  def __init__(self):
    self.motors = {}
    self.motors["FL"] = MotorController("P9_14")
    self.motors["FR"] = MotorController("P9_16")
    self.motors["TF"] = MotorController("P9_21")
    self.motors["TB"] = MotorController("P9_22")
    self.motors["LF"] = MotorController("P8_13") 
    self.motors["LB"] = MotorController("P8_19")
    sleep(1)

    for k in self.motors.keys():
        self.motors[k].set_power(0)
    rospy.Subscriber("rov_control", ControlData, self.callback)

  def callback(self, data):
    if data.stop:
      for motor in self.motors.values():
        motor.set_power(0)
    else:
      self.motors["FL"].set_power(data.forward_left) 
      self.motors["FR"].set_power(data.forward_right) 
      self.motors["TF"].set_power(data.top_front) 
      self.motors["TB"].set_power(data.top_back) 
      self.motors["LF"].set_power(data.lag_front) 
      self.motors["LB"].set_power(data.lag_back) 

if __name__ == '__main__':
    rospy.init_node('bottom_side', anonymous=True)
    core = RovController()

    while not rospy.is_shutdown():
        pass

    PWM.cleanup()
