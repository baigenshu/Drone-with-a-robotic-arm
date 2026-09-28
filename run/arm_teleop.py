#!/usr/bin/env python3
"""Keyboard teleoperation for the aerial-manipulator arm.

This is a more "real robot" friendly arm controller than arm_demo.py:

  * per-joint angle limits, clamped before anything is sent
  * slew-rate limiting (smooth interpolation) so the servos never jump --
    the commanded angles ramp towards the target instead of stepping
  * deadman behaviour: releasing keys holds the arm at the last command
  * works with a real terminal (cbreak mode) or with piped input (for tests)

Control chain (identical to the real-hardware path):

    this node  ->  /mavros/mount_control/command (mavros_msgs/MountControl)
               ->  PX4 gimbal module (MNT_MODE_IN=3)
               ->  MAVLink gimbal protocol v2 (GIMBAL_DEVICE_SET_ATTITUDE)
               ->  arm servo controller (Gazebo plugin in sim / MCU on real hw)

Keys
----
    a / d            base yaw      - / +
    w / s            arm roll      - / +
    q / e            gripper pitch - / +
    <space> or h     go to neutral (0, 0, 0)
    x  or Ctrl-C     quit

Joint limits (degrees), matching the Gazebo model
-------------------------------------------------
    yaw   : -90 .. +90    (joint itself is unlimited; software limit)
    roll  : -45 .. +45    (cgo3_horizontal_arm_joint hard limit)
    pitch : -60 .. +120   (cgo3_camera_joint hard limit)

Usage
-----
    # after run_sim.sh + run_mavros.sh, drone already flying:
    python3 arm_teleop.py

    # let it arm + take off first:
    python3 arm_teleop.py --takeoff

    # piped / scripted (no TTY):
    printf 'd\nd\nw\ne\nh\nx\n' | python3 arm_teleop.py
"""
import argparse
import math
import sys
import time

import rospy
from mavros_msgs.msg import MountControl, State
from mavros_msgs.srv import CommandBool, SetMode

# --- joint limits in degrees (see docstring) -------------------------------
LIMITS = {
    "yaw": (-90.0, 90.0),
    "roll": (-45.0, 45.0),
    "pitch": (-60.0, 120.0),
}
AXES = ("yaw", "roll", "pitch")

KEYMAP = {
    "a": ("yaw", -1.0),
    "d": ("yaw", +1.0),
    "w": ("roll", -1.0),
    "s": ("roll", +1.0),
    "q": ("pitch", -1.0),
    "e": ("pitch", +1.0),
}


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Keyboard:
    """Single-char reader: cbreak mode on a TTY, else line based on stdin."""

    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        self.fd = None
        self.old = None

    def __enter__(self):
        if self.is_tty:
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.is_tty and self.old is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        return False

    def get_key(self, timeout):
        """Return one key char, or None on timeout."""
        import select
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        if self.is_tty:
            return sys.stdin.read(1)
        # non-TTY (pipe/file): read a whole line at a time
        line = sys.stdin.readline()
        if line == "":
            return "x"          # EOF -> quit
        return line.strip()[:1] or None


def wait_for_mavros(timeout=30.0):
    rospy.loginfo("waiting for /mavros/state ...")
    try:
        rospy.wait_for_message("/mavros/state", State, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("MAVROS not reachable; start run_mavros.sh first")
        sys.exit(1)


def takeoff():
    rospy.wait_for_service("/mavros/cmd/arming", timeout=10.0)
    rospy.wait_for_service("/mavros/set_mode", timeout=10.0)
    arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    rospy.loginfo("arming ...")
    arm(True)
    time.sleep(3.0)
    rospy.loginfo("AUTO.TAKEOFF ...")
    mode(base_mode=0, custom_mode="AUTO.TAKEOFF")
    rospy.loginfo("climbing, waiting 20 s ...")
    time.sleep(20.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--takeoff", action="store_true",
                    help="arm and AUTO.TAKEOFF before starting teleop")
    ap.add_argument("--step", type=float, default=5.0,
                    help="degrees added per key press (default 5)")
    ap.add_argument("--max-rate", type=float, default=45.0,
                    help="max joint speed in deg/s for the smoothing (default 45)")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="publish rate in Hz (default 20)")
    # the sim mapping is not sign-symmetric; allow flipping per axis
    ap.add_argument("--invert-yaw", action="store_true")
    ap.add_argument("--invert-roll", action="store_true")
    ap.add_argument("--invert-pitch", action="store_true")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("arm_teleop", anonymous=False)

    if args.takeoff:
        takeoff()

    wait_for_mavros()

    pub = rospy.Publisher("/mavros/mount_control/command",
                          MountControl, queue_size=10)
    rospy.sleep(0.5)  # let the publisher connect

    sign = {
        "yaw": -1.0 if args.invert_yaw else 1.0,
        "roll": -1.0 if args.invert_roll else 1.0,
        "pitch": -1.0 if args.invert_pitch else 1.0,
    }

    target = [0.0, 0.0, 0.0]    # yaw, roll, pitch  [deg]
    current = [0.0, 0.0, 0.0]

    msg = MountControl()
    msg.header.frame_id = ""
    msg.mode = MountControl.MAV_MOUNT_MODE_MAVLINK_TARGETING  # 2
    msg.altitude = msg.latitude = msg.longitude = 0.0

    rospy.loginfo("keys: a/d=yaw  w/s=roll  q/e=pitch  space/h=neutral  x=quit")
    rospy.loginfo("")
    rospy.loginfo(">>> 请先点击本终端窗口，使它获得焦点，然后再按键 <<<")
    if not sys.stdin.isatty():
        rospy.logwarn("stdin 不是终端：将按行读取，每行第一个字符当作一次按键")

    rate = rospy.Rate(args.rate)
    dt = 1.0 / args.rate
    max_step = args.max_rate * dt
    last_print = 0.0
    start_time = time.time()
    got_key = False
    warned_no_key = False

    with Keyboard() as kb:
        while not rospy.is_shutdown():
            key = kb.get_key(dt)
            if key:
                got_key = True
                k = key.lower()
                if k in ("x", "\x03"):           # x or Ctrl-C
                    break
                elif k in (" ", "h"):
                    target = [0.0, 0.0, 0.0]
                    rospy.loginfo("-> neutral")
                elif k in KEYMAP:
                    axis, d = KEYMAP[k]
                    i = AXES.index(axis)
                    lo, hi = LIMITS[axis]
                    target[i] = clamp(target[i] + d * args.step, lo, hi)
                    rospy.loginfo("target  yaw=%6.1f roll=%6.1f pitch=%6.1f (deg)",
                                  target[0], target[1], target[2])

            if not got_key and not warned_no_key and (time.time() - start_time) > 10.0:
                rospy.logwarn("已 10 秒未检测到任何按键。键盘事件只发给有焦点的窗口："
                              "请用鼠标点一下本终端，再按 a/d/w/s/q/e。")
                warned_no_key = True

            # slew-rate limited interpolation towards the target
            for i in range(3):
                diff = target[i] - current[i]
                if abs(diff) <= max_step:
                    current[i] = target[i]
                else:
                    current[i] += math.copysign(max_step, diff)

            msg.header.stamp = rospy.Time.now()
            msg.yaw = current[0] * sign["yaw"]
            msg.roll = current[1] * sign["roll"]
            msg.pitch = current[2] * sign["pitch"]
            pub.publish(msg)

            now = time.time()
            if now - last_print > 2.0:
                rospy.loginfo("current yaw=%6.1f roll=%6.1f pitch=%6.1f (deg)",
                              current[0], current[1], current[2])
                last_print = now

            rate.sleep()

    rospy.loginfo("arm_teleop stopped")


if __name__ == "__main__":
    main()
