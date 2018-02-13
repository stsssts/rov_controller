#!/usr/bin/env python
import rospy
import math

from bridge import ArduinoBoard
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState


class RovController:
    def __init__(self):
        self.arduino = ArduinoBoard(self.parse_message)
        self.thrusters = {'port': 0.0, 'starboard': 0.0, 'vertical': 0.0}
        self.light_intensity = 0
        self.camera_position = 0

        rospy.Subscriber('thrusters/port', Float64, self.thrusters_callback, 'port')
        rospy.Subscriber('thrusters/starboard', Float64, self.thrusters_callback, 'starboard')
        rospy.Subscriber('thrusters/vertical', Float64, self.thrusters_callback, 'vertical')

        rospy.Subscriber('lights/inc', UInt8, self.lights_callback, 'inc')
        rospy.Subscriber('lights/dec', UInt8, self.lights_callback, 'dec')

        rospy.Subscriber('camera_tilt/up', Float64, self.camera_tilt_callback, 'up')
        rospy.Subscriber('camera_tilt/down', Float64, self.camera_tilt_callback, 'down')

        self.pose_publisher = rospy.Publisher('pose', TwistStamped, queue_size=1)
        self.battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)

    def thrusters_callback(self, msg, thruster):
        if abs(msg.data) <= 100.0:
            self.thrusters[thruster] = msg.data
        else:
            rospy.logerr('Invalid value for %s thruster power!' % thruster)

    def lights_callback(self, msg, cmd):
        self.light_intensity += msg.data if cmd == 'up' else -msg.data
        if self.light_intensity > 255:
            self.light_intensity = 255
        if self.light_intensity < 0:
            self.light_intensity = 0
        self.arduino.send('ligt(%s);' % self.light_intensity)

    def camera_tilt_callback(self, msg, cmd):
        self.camera_position += msg.data if cmd == 'up' else -msg.data
        if abs(msg.data) > 100.0:
            msg.data = math.copysign(100.0, msg.data)
        zero_pos_microsecs = 1487.0
        microsecs_per_degree = 9.523809
        target = self.map(msg.data, min_out=-41.7, max_out=31.8)
        servo_pos = (microsecs_per_degree * target) + zero_pos_microsecs
        self.arduino.send('tilt(%i);' % servo_pos)

    def update_thrusters(self):
        p = self.map(float(self.thrusters['port']))
        v = self.map(float(self.thrusters['vertical']))
        s = self.map(float(self.thrusters['starboard']))
        self.arduino.send('go(%i,%i,%i);' % (p, v, s))

    def map(self, power, min_in=-100.0, max_in=100.0, min_out=1000.0, max_out=2000.0):
        return (power - min_in) * (max_out - min_out) / (max_in - min_in) + min_out

    def update_status(self, msg):
        status = {}
        for param in msg.split(';'):
            param = param.split(':')
            if len(param) == 2:
                status[param[0]] = param[1]
        if 'roll' in status and 'pitc' in status and 'yaw' in status:
            pose = TwistStamped()
            pose.header.stamp = rospy.Time.now()
            pose.twist.linear.z = float(status['deap'])
            pose.twist.angular.x = float(status['roll'])
            pose.twist.angular.y = float(status['pitc'])
            pose.twist.angular.z = float(status['yaw'])
            self.pose_publisher.publish(pose)
        # TODO: get battery info from msg and publish to appropriate topics

    def parse_message(self, msg):
        if 'roll:' in msg:
            self.update_status(msg)
        # TODO: parse battery message

    def ping(self, _):
        self.arduino.send('ping(0);')


if __name__ == '__main__':
    rospy.init_node('bottom_side', anonymous=True)
    core = RovController()

    rate = rospy.Rate(10) # 10hz
    rospy.Timer(rospy.Duration(1), core.ping)

    while not rospy.is_shutdown():
        core.update_thrusters()
        rate.sleep()
