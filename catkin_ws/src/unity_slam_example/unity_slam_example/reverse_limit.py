"""Keep move_base's reversing short: /cmd_vel_dwa -> /cmd_vel_nav (-> cmd_vel_guard).

DWA may back away slowly (min_vel_x < 0), but nothing in its scoring prefers driving forward: with
the goal behind the robot it reverses all the way there, at 0.05 m/s. This node lets each stretch
of reversing cover at most `budget` metres; after that it turns in place, in one direction, until
DWA drives forward again (which restores the budget). Assisted driving publishes straight to
/cmd_vel_nav, so a driver's reverse is not limited.
"""

import rospy
from geometry_msgs.msg import Twist


class ReverseBudget:
    def __init__(self, budget=0.15, turn_rate=0.4, forward=0.03, max_dt=0.2):
        self.budget = budget        # metres of reversing per stretch
        self.turn_rate = turn_rate  # rad/s once the budget is spent
        self.forward = forward      # m/s: a forward command this fast ends the stretch
        self.max_dt = max_dt        # s: a gap between commands counts as at most this
        self.used = 0.0
        self.direction = 0

    def __call__(self, v, w, dt):
        if v >= self.forward:
            self.used, self.direction = 0.0, 0
        elif v < 0.0:
            step = -v * min(max(dt, 0.0), self.max_dt)   # a clock jumping back counts 0
            if self.used + step > self.budget + 1e-9:   # float sums of equal steps overshoot
                if not self.direction:
                    self.direction = 1 if w >= 0.0 else -1
                return 0.0, self.direction * self.turn_rate
            self.used += step
        return v, w


class ReverseLimit:
    def __init__(self):
        self.limit = ReverseBudget(rospy.get_param('~budget', 0.15),
                                   rospy.get_param('~turn_rate', 0.4))
        self.last = None
        self.publisher = rospy.Publisher('cmd_vel_nav', Twist, queue_size=10)
        rospy.Subscriber('cmd_vel_dwa', Twist, self.on_cmd, queue_size=10)

    def on_cmd(self, cmd):
        now = rospy.Time.now()
        dt = 0.0 if self.last is None else (now - self.last).to_sec()
        self.last = now
        out = Twist()
        out.linear.x, out.angular.z = self.limit(cmd.linear.x, cmd.angular.z, dt)
        self.publisher.publish(out)


def main():
    rospy.init_node('reverse_limit')
    ReverseLimit()
    rospy.spin()
