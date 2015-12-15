#!/usr/bin/env python

import math
import time

import Adafruit_BBIO.PWM as PWM

import rospy
from rov_control.msg import ControlData

class MotorController():
  def __init__(self, port):
    self.MAX = 0.25
    self.stop_time  = 0.0015
    self.time_delta = 0.0004
    self.port = port
    self.frequency = 100
    self.period = 1.0/self.frequency
    PWM.stop(port)

  def start(self):
    PWM.start(self.port, 100*self.stop_time/self.period, self.frequency)

  def set_power(self, power):
    if abs(power) > self.MAX:
      power = math.copysign(self.MAX, power)
    time = self.stop_time + power*self.time_delta
    percent = 100*time/self.period
    print self.port, percent
    PWM.set_duty_cycle(self.port, percent)

  def cleanup(self):
    PWM.stop(self.port)

class RovController:
  def __init__(self):
    self.motors = {}
    self.motors["FL"] = MotorController("P8_13") #P2B
    self.motors["FR"] = MotorController("P9_16")
    """
    self.motors["TF"] = MotorController("P9_21")
    self.motors["TB"] = MotorController("P9_22")
    self.motors["LF"] = MotorController("P9_42") 
    self.motors["LB"] = MotorController("P9_14")
    """
    PWM.cleanup()
    for motor in self.motors.values():
      motor.start()

    rospy.init_node('bottom_side', anonymous=True)
    rospy.Subscriber("rov_control", ControlData, self.callback)

  def __exit__(self, exc_type, exc_value, traceback):
    for motor in self.motors.values():
      motor.cleanup()
    PWM.cleanup()

  def callback(self, data):
    print data 
    if data.stop:
      for motor in self.motors.values():
        motor.set_power(0)
    else:
      self.motors["FL"].set_power(data.forward_left) 
      self.motors["FR"].set_power(data.forward_right) 
"""
      self.motors["TF"].set_power(data.top_front) 
      self.motors["TB"].set_power(data.top_back) 
      self.motors["LF"].set_power(data.lag_front) 
      self.motors["LB"].set_power(data.lag_back) 
"""
if __name__ == '__main__':
    core = RovController()

    while True:
        pass
