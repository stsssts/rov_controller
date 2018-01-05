class Thruster:
    def __init__(self, settings):
        self.reversed = settings['reversed']
        self.forward_modifier = settings['forward_modifier']
        self.reverse_modifier = settings['reverse_modifier']

    def get_pwm(self, value):
        if self.reversed:
            value *= -1
        if value > 100:
            value = 100
        elif value < -100:
            value = -100
        value *= self.forward_modifier if value > 0 else self.reverse_modifier
        return int((value + 100) * 1000 / 200 + 1000)

    def __repr__(self):
        return '\nFw mod: %s\nRv mod: %s\n' % (self.forward_modifier, self.reverse_modifier)

class ThrustersController:
    def __init__(self):
        self.thrusters = {}

    def configure(self, settings):
        for thr in settings:
            self.thrusters[thr] = Thruster(settings[thr])

    def get_pwm2x1(self, throttle, lift, yaw):
        p = self.thrusters['port'].get_pwm(throttle + yaw)
        s = self.thrusters['starboard'].get_pwm(throttle - yaw)
        v = self.thrusters['vertical'].get_pwm(lift)
        return (p, v, s)
