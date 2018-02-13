#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


class Thruster:
    # TODO: add curve interpolation
    def __init__(self, settings):
        self.reversed = settings['reversed']
        self.forward_modifier = settings['forward_modifier']
        self.reverse_modifier = settings['reverse_modifier']

    def get_effort(self, cmd):
        if self.reversed:
            cmd *= -1
        cmd *= self.forward_modifier if cmd > 0 else self.reverse_modifier
        return cmd


class ThrustersController:
    def __init__(self):
        # TODO: make generalized thruster controller
        if not rospy.has_param('thrusters'):
            rospy.logerr("Didn't find thrusters configuration")
            raise rospy.ROSException

        self.thrusters = {}
        self.configure(rospy.get_param('thrusters'))
        self.command = {'surge': 0.0, 'heave': 0.0, 'yaw': 0.0}

        rospy.Subscriber('thrust_command/surge', Float64, self.command_callback, 'surge')
        rospy.Subscriber('thrust_command/heave', Float64, self.command_callback, 'heave')
        rospy.Subscriber('thrust_command/yaw', Float64, self.command_callback, 'yaw')

        self.pub_port = rospy.Publisher('thrusters/port', Float64, queue_size=1)
        self.pub_starboard = rospy.Publisher('thrusters/starboard', Float64, queue_size=1)
        self.pub_vertical = rospy.Publisher('thrusters/vertical', Float64, queue_size=1)

    def configure(self, settings):
        for thr in settings:
            self.thrusters[thr] = Thruster(settings[thr])

    def command_callback(self, msg, cmd):
        self.command[cmd] = msg.data

    def update(self):
        port_cmd = self.command['surge'] + self.command['yaw']
        vertical_cmd = self.command['heave']
        starboard_cmd = self.command['surge'] - self.command['yaw']

        port = self.thrusters['port'].get_effort(port_cmd)
        vertical = self.thrusters['vertical'].get_effort(vertical_cmd)
        starboard = self.thrusters['starboard'].get_effort(starboard_cmd)

        self.pub_port.publish(port)
        self.pub_vertical.publish(vertical)
        self.pub_starboard.publish(starboard)


if __name__ == '__main__':
    rospy.init_node('thrusters_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    thrusters_controller = ThrustersController()
    while not rospy.is_shutdown():
        thrusters_controller.update()
        rate.sleep()