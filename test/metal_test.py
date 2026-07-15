#!/usr/bin/env python3
"""Metal arm hardware test helper (metal_sdk direct, no ROS).

Subcommands, safety-graded:
  read     READ-ONLY   — read joint state, arm never energized.
  gravity  ENERGIZES   — gravity compensation teach mode; arm holds its weight,
                         back-drivable by hand. No autonomous motion.
  hold     ENERGIZES   — enable + command CURRENT position; arm should NOT move.
  move     MOVES       — small deliberate single-joint move, then back.

Run via the t*.sh wrappers (they set up the ROS/py env). Uses the DRIVER urdf
(capital link names Link1..Link6) which is what the SDK's kdl_solver requires.
"""
import argparse
import os
import sys
import time

from metal_sdk import MetalSDKInterface, ControlMode

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.abspath(os.path.join(
    HERE, "..", "src", "metal_arm_driver", "urdf", "metal_no_gripper.urdf"))
CAN = "can0"
END_TYPE = 0  # 0 = no gripper


def confirm(msg, yes):
    print("\n*** SAFETY CHECK ***")
    print(msg)
    print("Keep a hand on the e-stop and clear the arm's workspace.")
    if yes:
        print("(--yes supplied, proceeding)")
        return
    try:
        if input("Type 'yes' to proceed: ").strip() != "yes":
            print("aborted."); sys.exit(0)
    except EOFError:
        print("no interactive input -> aborting for safety."); sys.exit(1)


def make_arm(enable):
    print("urdf:", URDF, "| exists:", os.path.exists(URDF))
    arm = MetalSDKInterface(CAN, URDF, END_TYPE, enable)
    if not arm.Init():
        print("[FATAL] Init() failed — check CAN up, arm powered, bitrate 1Mbps.")
        sys.exit(1)
    time.sleep(0.5)
    return arm


def dump(arm):
    print("  joint names :", arm.GetJointNames())
    print("  joint pos   :", [round(x, 4) for x in arm.GetJointPosition()])
    print("  joint vel   :", [round(x, 4) for x in arm.GetJointVelocity()])
    print("  error codes :", arm.GetJointErrorCode())
    print("  rotor temp  :", arm.GetRotorTemperature())
    print("  end pose    :", [round(x, 4) for x in arm.GetArmEndPose()])


def wait_reach(arm, target, tol=0.03, timeout=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        q = arm.GetJointPosition()
        err = max(abs(q[i] - target[i]) for i in range(6))
        print("    err=%.3f  q=%s" % (err, [round(x, 3) for x in q[:6]]))
        if err < tol:
            print("    reached."); return
        time.sleep(0.4)
    print("    timeout — arm may still be settling.")


def safe_park(arm):
    print("\nLeaving arm in GRAVITY_COMPENSATION (safe, back-drivable).")
    arm.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
    time.sleep(0.5)


def cmd_read(args):
    print("== READ-ONLY: arm will NOT be energized ==")
    arm = make_arm(enable=False)
    dump(arm)
    print("\nDONE. If joint pos changes when you nudge the arm by hand, CAN<->arm works.")


def cmd_gravity(args):
    confirm("GRAVITY COMPENSATION teach mode: the arm ENERGIZES and holds its own "
            "weight; you can move it by hand. It will NOT move on its own.", args.yes)
    arm = make_arm(enable=True)
    arm.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
    print("\nGravity-comp ON. Move the arm by hand — joint angles print live.")
    print("Press Ctrl-C to stop.\n")
    try:
        while True:
            print("q:", [round(x, 3) for x in arm.GetJointPosition()],
                  "| end:", [round(x, 3) for x in arm.GetArmEndPose()])
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\nstopping.")


def cmd_hold(args):
    confirm("HOLD test: the arm ENERGIZES and is commanded to its CURRENT position. "
            "It should stay put (maybe a tiny settle). No deliberate motion.", args.yes)
    arm = make_arm(enable=True)
    q = [float(x) for x in arm.GetJointPosition()][:6]
    print("holding at:", [round(x, 3) for x in q])
    arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
    arm.SetArmJointPosition(q, 1)  # velocity_ratio=1 (slowest)
    print("\nHolding position (rigid). 'drift' = deviation from the commanded pose;")
    print("small/steady drift means it's holding well. SUPPORT the arm, then Ctrl-C")
    print("to release (it returns to gravity-comp, which does NOT fully hold weight).\n")
    try:
        while True:
            cur = arm.GetJointPosition()
            drift = max(abs(cur[i] - q[i]) for i in range(6))
            print("drift=%.4f  q=%s" % (drift, [round(x, 3) for x in cur[:6]]))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nreleasing.")
    safe_park(arm)


def cmd_move(args):
    j, d = args.joint, args.delta
    confirm("MOVE test: the arm ENERGIZES and joint J%d moves by %+.3f rad (%.1f deg), "
            "then returns. Small & slow, but it WILL move." % (j + 1, d, d * 57.3), args.yes)
    arm = make_arm(enable=True)
    q0 = [float(x) for x in arm.GetJointPosition()][:6]
    print("start:", [round(x, 3) for x in q0])
    arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
    tgt = list(q0); tgt[j] += d
    print("moving J%d to %.3f (velocity_ratio=1, slowest)..." % (j + 1, tgt[j]))
    arm.SetArmJointPosition(tgt, 1)
    wait_reach(arm, tgt)
    print("returning to start...")
    arm.SetArmJointPosition(q0, 1)
    wait_reach(arm, q0)
    safe_park(arm)


def main():
    p = argparse.ArgumentParser(description="Metal arm hardware tests")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("read").set_defaults(func=cmd_read)
    g = sub.add_parser("gravity"); g.set_defaults(func=cmd_gravity)
    h = sub.add_parser("hold"); h.set_defaults(func=cmd_hold)
    m = sub.add_parser("move"); m.set_defaults(func=cmd_move)
    m.add_argument("--joint", type=int, default=0, help="joint index 0..5 (default 0=J1)")
    m.add_argument("--delta", type=float, default=0.15, help="radians (default 0.15 ~8.6deg)")
    for s in (g, h, m):
        s.add_argument("--yes", action="store_true", help="skip the confirm prompt")
    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
