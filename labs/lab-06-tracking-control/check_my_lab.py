"""
Lab 6: Path Tracking
Validation Script
Author: Dr. Akram Fatayer

Tests the student's Pure Pursuit implementation against known geometric cases.
"""

import numpy as np
import sys
import os


def check(name, passed, detail=""):
    prefix = "[OK]" if passed else "[!!]"
    status = "PASS" if passed else "FAIL"
    print(f"  {prefix} {status} | {name}")
    if detail and not passed:
        print(f"         -> {detail}")
    return passed


def run_tests():
    print("=" * 55)
    print("  Lab 6 Validation: Pure Pursuit Controller")
    print("=" * 55)

    if not os.path.exists("activity2_pure_pursuit.py"):
        print("  ERROR: activity2_pure_pursuit.py not found.")
        print("  Run this script from the Lab6_PathTracking directory.")
        sys.exit(1)

    try:
        from activity2_pure_pursuit import pure_pursuit_control
    except ImportError as e:
        print(f"  ERROR importing pure_pursuit_control: {e}")
        sys.exit(1)

    L = 2.5
    results = []

    # Test 1: Straight ahead -> zero steering
    try:
        steer = pure_pursuit_control(np.array([0.0, 0.0, 0.0]),
                                     np.array([5.0, 0.0]), 5.0, L)
        results.append(check("Straight line -> 0 steering",
                             abs(steer) < 1e-5,
                             f"expected 0.0, got {steer:.4f}"))
    except Exception as e:
        results.append(check("Straight line", False, f"error: {e}"))

    # Test 2: 90-degree target to the left
    # alpha = pi/2, steer = arctan(2*2.5*1 / 5) = arctan(1) = pi/4
    try:
        steer    = pure_pursuit_control(np.array([0.0, 0.0, 0.0]),
                                        np.array([0.0, 5.0]), 5.0, L)
        expected = np.pi / 4.0
        results.append(check("90-deg curve -> pi/4",
                             abs(steer - expected) < 1e-4,
                             f"expected {expected:.4f}, got {steer:.4f}"))
    except Exception as e:
        results.append(check("90-deg curve", False, f"error: {e}"))

    # Test 3: Larger Ld -> gentler steering
    try:
        steer    = pure_pursuit_control(np.array([0.0, 0.0, 0.0]),
                                        np.array([0.0, 10.0]), 10.0, L)
        expected = np.arctan(0.5)
        results.append(check("Ld sensitivity (Ld=10)",
                             abs(steer - expected) < 1e-4,
                             f"expected {expected:.4f}, got {steer:.4f}"))
    except Exception as e:
        results.append(check("Ld sensitivity", False, f"error: {e}"))

    # Test 4: Symmetry -- target to the right gives mirror-image steering
    try:
        s_left  = pure_pursuit_control(np.array([0.0, 0.0, 0.0]),
                                       np.array([0.0,  5.0]), 5.0, L)
        s_right = pure_pursuit_control(np.array([0.0, 0.0, 0.0]),
                                       np.array([0.0, -5.0]), 5.0, L)
        results.append(check("Left/right symmetry",
                             abs(s_left + s_right) < 1e-5,
                             f"expected opposite signs, got {s_left:.4f}, {s_right:.4f}"))
    except Exception as e:
        results.append(check("Symmetry", False, f"error: {e}"))

    print("-" * 55)
    passed = sum(results)
    total  = len(results)
    if passed == total:
        print(f"  SUCCESS: all {total} tests passed.")
        print("  Your Pure Pursuit math is correct -- ready to submit.")
    else:
        print(f"  {passed}/{total} tests passed.")
        print("  Check your pure_pursuit_control() implementation.")
        print("  Hint: use np.arctan2() and normalize alpha to [-pi, pi].")


if __name__ == "__main__":
    run_tests()
