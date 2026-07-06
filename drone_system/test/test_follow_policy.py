"""Unit tests for the follower's core policy (follow_policy.py).

These test the SAME pure functions the follower node imports and flies, so the
tested logic is the real logic. No ROS / sim needed -- runs under plain pytest.
"""

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "drone_system"))
from follow_policy import distance, exceeds_jump, follow_waypoint  # noqa: E402


def test_jump_gate_rejects_teleports_keeps_small_steps():
    assert exceeds_jump((10.0, 0.0), (0.0, 0.0), 5.0)        # 10 m step > 5 m -> reject
    assert not exceeds_jump((3.0, 0.0), (0.0, 0.0), 5.0)     # 3 m step < 5 m -> accept
    assert not exceeds_jump((0.0, 0.0), (0.0, 0.0), 5.0)     # no motion
    assert distance((3.0, 4.0), (0.0, 0.0)) == 5.0           # 3-4-5


def test_waypoint_is_offset_behind_when_no_leadtime():
    # car at origin, heading +x, offset 5, lead 0 -> 5 m behind at (-5, 0, 20)
    wp = follow_waypoint((0.0, 0.0), (0.0, 0.0), (1.0, 0.0), 5.0, 0.0, 20.0)
    assert abs(wp[0] + 5.0) < 1e-9 and abs(wp[1]) < 1e-9 and wp[2] == 20.0


def test_feedforward_pulls_target_toward_the_car():
    # car at +2.5 m/s, lead 1.5 -> projected +3.75; minus 5 offset -> -1.25:
    # the target sits closer to the car than the plain -5, compensating drone lag.
    wp = follow_waypoint((0.0, 0.0), (2.5, 0.0), (1.0, 0.0), 5.0, 1.5, 20.0)
    assert abs(wp[0] + 1.25) < 1e-9


def test_faster_car_projects_target_further_forward():
    slow = follow_waypoint((0.0, 0.0), (1.0, 0.0), (1.0, 0.0), 5.0, 1.5, 20.0)
    fast = follow_waypoint((0.0, 0.0), (4.0, 0.0), (1.0, 0.0), 5.0, 1.5, 20.0)
    assert fast[0] > slow[0]        # more speed -> more forward projection


def test_altitude_passthrough():
    assert follow_waypoint((3.0, 3.0), (0.0, 0.0), (0.0, 1.0), 5.0, 0.0, 20.0)[2] == 20.0


if __name__ == "__main__":
    ok = True
    for fn in (test_jump_gate_rejects_teleports_keeps_small_steps,
               test_waypoint_is_offset_behind_when_no_leadtime,
               test_feedforward_pulls_target_toward_the_car,
               test_faster_car_projects_target_further_forward,
               test_altitude_passthrough):
        try:
            fn()
            print(f"  [PASS] {fn.__name__}")
        except AssertionError as e:
            ok = False
            print(f"  [FAIL] {fn.__name__}: {e}")
    print("SELF-TEST:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
