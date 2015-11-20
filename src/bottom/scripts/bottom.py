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
    PWM.set_duty_cycle(self.port, percent)

  def cleanup(self):
    PWM.stop(self.port)

class RovController:
  def __init__(self):
    self.motors = {}
    self.motors["FL"] = MotorController("P9_14")
    self.motors["FR"] = MotorController("P9_16")
    self.motors["TL"] = MotorController("P9_22")
    self.motors["TR"] = MotorController("P9_28")
    self.motors["R0"] = MotorController("P9_42")
    self.motors["R1"] = MotorController("P8_13")
    self.motors["R2"] = MotorController("P8_19")

    PWM.cleanup()
    for motor in self.motors.values():
      motor.start()

    rospy.init_node('bottom_side', anonymous=True)
    rospy.Subscriber("rov_controller", ControlData, self.callback)

  def __exit__(self, exc_type, exc_value, traceback):
    for motor in self.motors.values():
      motor.cleanup()
    PWM.cleanup()


  def callback(self, data):
    message = ControlData()
    if message.stop:
      for motor in self.motors.value():
        motor.set_power(0)
      else:
        self.motors["FL"].set_power(data.forward_left) 
        self.motors["FR"].set_power(data.forward_right) 
        self.motors["TL"].set_power(data.top_left) 
        self.motors["TR"].set_power(data.top_right) 
    
if __name__ == '__main__':
    core = RovController()
    while True:
        pass
