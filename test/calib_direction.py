#!/usr/bin/env python3
"""Per-joint DIRECTION check for gravity-comp calibration (READ-ONLY, motors OFF).

Physics: a free (limp) joint sags toward LOWER potential energy, i.e. in the
-sign(G[j]) direction (URDF convention), where G(q) is the model gravity torque
(SDK.ComputeGravityTorque). For each gravity-loaded joint you gently move it the
way gravity pulls it; the script compares the observed motor-position change
sign to -sign(G[j]):

  observed == -sign(G)  -> motor direction agrees with the URDF (CONSISTENT)
  observed == +sign(G)  -> motor direction is FLIPPED vs the URDF  <-- the bug

A FLIPPED joint has its gravity torque computed with the wrong sign, which is
exactly why gravity-comp cannot hold it. J1 (and any near-vertical-axis joint)
is gravity-neutral (G~0) and cannot be checked this way — use a kinematic check.

Usage: bash test/tc_direction.sh          (all joints)
       bash test/tc_direction.sh --joint 2  (just J2)
"""
import argparse
import os
import sys
import time

from metal_sdk import MetalSDKInterface

URDF = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..",
    "src", "metal_arm_driver", "urdf", "metal_no_gripper.urdf"))

G_MIN = 0.3    # Nm: below this the joint is ~gravity-neutral in this pose
DQ_MIN = 0.02  # rad: minimum motion to be conclusive


def check_joint(arm, j):
    jn = j + 1
    print("\n===== J%d =====" % jn)
    input("  Arrange the arm so J%d is clearly gravity-loaded and free to move, "
          "then press Enter..." % jn)
    q0 = list(arm.GetJointPosition())[:6]
    g0 = list(arm.ComputeGravityTorque(q0))[:6]
    gj = g0[j]
    print("  pose q[J%d]=%.3f | model gravity G[J%d]=%+.3f Nm" % (jn, q0[j], jn, gj))
    if abs(gj) < G_MIN:
        print("  -> |G[J%d]| ~ 0 (gravity-neutral in this pose). Re-pose it more "
              "horizontal, or J%d may be a vertical-axis joint that gravity can't "
              "test. SKIPPED." % (jn, jn))
        return "skipped (gravity-neutral)"
    input("  Now GENTLY move J%d the way GRAVITY PULLS IT (let it droop down) a "
          "clear amount, hold, then press Enter..." % jn)
    q1 = list(arm.GetJointPosition())[:6]
    dq = q1[j] - q0[j]
    if abs(dq) < DQ_MIN:
        print("  -> J%d barely moved (dq=%+.3f). Move it more; INCONCLUSIVE." % (jn, dq))
        return "inconclusive (motion too small)"
    expected = -1 if gj > 0 else 1   # free joint sags toward -sign(G)
    observed = 1 if dq > 0 else -1
    ok = (expected == observed)
    verdict = "CONSISTENT (direction OK)" if ok else \
              "*** FLIPPED — motor direction opposite to URDF ***"
    print("  dq[J%d]=%+.3f | expected sag sign=%+d, observed=%+d  ->  %s"
          % (jn, dq, expected, observed, verdict))
    return verdict


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--joint", type=int, default=None,
                   help="check only this joint (1..6); default all")
    args = p.parse_args()

    arm = MetalSDKInterface("can0", URDF, 0, False)  # enable=False -> motors OFF
    if not arm.Init():
        print("[FATAL] Init failed — CAN up? arm powered?")
        sys.exit(1)
    time.sleep(0.5)
    print("Motors OFF (arm is limp/back-drivable). READ-ONLY direction check.")

    joints = [args.joint - 1] if args.joint else range(6)
    results = {}
    for j in joints:
        results[j + 1] = check_joint(arm, j)

    print("\n========== SUMMARY ==========")
    for jn, v in results.items():
        print("  J%d: %s" % (jn, v))
    flipped = [jn for jn, v in results.items() if "FLIPPED" in v]
    if flipped:
        print("\nFLIPPED joints: %s" % flipped)
        print("Their gravity torque is computed with the wrong sign -> gravity-comp")
        print("can't hold them. Fix by flipping that joint's axis in the URDF (or")
        print("re-zeroing / adding a sign map), then re-run this check.")
    else:
        print("\nNo flips detected among the checked joints.")


if __name__ == "__main__":
    main()
