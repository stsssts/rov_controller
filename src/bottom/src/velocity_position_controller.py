#!/usr/bin/env python
from math import copysign, atan2, sin, cos, pi

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

from dynamic_reconfigure.server import Server
from bottom.cfg import PidConfig


class PID:
    def __init__(self):
        self.e_sum = self.e_old = 0
        self.kp = self.ki = self.kd = 0

    def set_params(self, settings):
        rospy.loginfo('Set Pid parameters.')
        self.kp = settings['kp']
        self.ki = settings['ki']
        self.kd = settings['kd']
        self.clamp = settings['clamp']

    def update(self, error):
        p = self.kp * error
        self.e_sum += self.ki * error
        if abs(self.e_sum) > self.clamp:
            self.e_sum = copysign(self.clamp, self.e_sum)
        d = self.kd * (error - self.e_old)
        self.e_old = error
        return p + self.e_sum + d


class VelocityController:
    def __init__(self):
        rospy.Subscriber('angular_velocity_command', Float64, self.command_callback)
        rospy.Subscriber('pose', TwistStamped, self.pose_callback)

        self.thrust_publisher = rospy.Publisher('thrust_command/yaw', Float64, queue_size=1)

        self.desired_pose = 0.0
        self.desired_speed = 0.0

        self.pid = PID()
        self.dynamic_reconfigure_srv = Server(PidConfig, self.dynamic_reconfigure_callback)
        self.pid.set_params(rospy.get_param('yaw_pid'))
        self.prev_time = rospy.Time.now()
        # Needed parameters:
        # 1. PID coefficients
        # 2. clamp
        # 3. time between messages
        # 4. maximum velocity
        # 5. linear or angular control

    def command_callback(self, msg):
        self.desired_speed = msg.data

    def pose_callback(self, msg):
        state = msg.twist.angular.z
        error = (state - self.desired_pose) * pi / 180.0
        error = atan2(sin(error), cos(error))  # error -> [-pi, pi]
        control_effort = self.pid.update(error)

        self.thrust_publisher.publish(control_effort)

    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {kp}, {ki}, {kd}, {clamp}""".format(**config))
        self.pid.set_params(config)
        return config

    def update(self):
        curr_time = rospy.Time.now()
        self.desired_pose += self.desired_speed * (curr_time - self.prev_time).to_sec()
        self.prev_time = curr_time
        # TODO: constrain desired speed


if __name__ == '__main__':
    rospy.init_node('velocity_position_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    vel_controller = VelocityController()
    while not rospy.is_shutdown():
        vel_controller.update()
        rate.sleep()