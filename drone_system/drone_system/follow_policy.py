"""follow_policy -- the follower's pure decision logic, isolated for unit tests.

No ROS / rclpy imports: the geometry and gates the `follower` node relies on, as
plain functions, so they can be unit-tested without a running sim
(see test/test_follow_policy.py). The node imports and uses these directly, so the
tested logic is the SAME logic that flies.
"""

import math


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def exceeds_jump(new_xy, last_xy, threshold):
    """True if a single position step is an implausible teleport (the > jump gate)."""
    return distance(new_xy, last_xy) > threshold


def follow_waypoint(car_xy, car_vel, heading, offset, lead_time, altitude):
    """The follow policy, as a pure function.

    Project the car forward by lead_time * velocity (velocity feed-forward, to
    cancel the drone's own position-tracking lag), then step `offset` metres back
    along the travel `heading`, at `altitude`. lead_time = 0 recovers the plain
    geometric 'offset behind the current reported position'.

    Why a *fixed* lead_time is speed-correct: the drone's steady-state trailing
    lag on a ramp input is ~ v / K (proportional to car speed v), and the lead
    distance is v * lead_time (also proportional to v). One constant lead_time
    therefore cancels the lag across speeds -- it is not a per-speed magic number.
    """
    px = car_xy[0] + car_vel[0] * lead_time
    py = car_xy[1] + car_vel[1] * lead_time
    return (px - offset * heading[0], py - offset * heading[1], altitude)
