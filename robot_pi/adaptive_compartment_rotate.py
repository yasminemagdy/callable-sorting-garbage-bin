#!/usr/bin/env python
# -*- coding: ascii -*-

import rospy
import math
import sys
import os

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

# Change these if your physical compartment angles are different.
# recyclable is the reference position facing the user.
COMPARTMENT_OFFSETS_DEG = {
    "recyclable": 0.0,
    "paper": 120.0,
    "general_waste": -120.0
}

BASE_FILE = os.path.expanduser("~/.compartment_base_yaw.txt")
STATE_FILE = os.path.expanduser("~/.current_compartment.txt")

KP = 1.5
MAX_W = 0.45
TOL_DEG = 3.0
SETTLED_COUNT_REQUIRED = 10
TIMEOUT_SEC = 30.0

current_yaw = None


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def amcl_callback(msg):
    global current_yaw
    q = msg.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    current_yaw = yaw


def wait_for_yaw():
    print("Waiting for /amcl_pose...")
    start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and current_yaw is None:
        if rospy.Time.now().to_sec() - start > 10.0:
            print("ERROR: No /amcl_pose received. Check AMCL first.")
            sys.exit(1)
        rospy.sleep(0.1)


def save_base_yaw():
    wait_for_yaw()
    with open(BASE_FILE, "w") as f:
        f.write(str(current_yaw))
    with open(STATE_FILE, "w") as f:
        f.write("recyclable")
    print("SAVED recyclable/user-facing yaw.")
    print("Base yaw deg:", round(math.degrees(current_yaw), 2))
    print("Current compartment: recyclable")


def load_base_yaw():
    if not os.path.exists(BASE_FILE):
        print("ERROR: Base yaw not saved yet.")
        print("First place recyclable facing the user, then run:")
        print("python ~/adaptive_compartment_rotate.py calibrate")
        sys.exit(1)
    with open(BASE_FILE, "r") as f:
        return float(f.read().strip())


def get_current_compartment():
    if not os.path.exists(STATE_FILE):
        return "unknown"
    with open(STATE_FILE, "r") as f:
        return f.read().strip()


def save_current_compartment(name):
    with open(STATE_FILE, "w") as f:
        f.write(name)


def rotate_to_compartment(target_name):
    if target_name not in COMPARTMENT_OFFSETS_DEG:
        print("ERROR: Invalid compartment.")
        print("Use one of: recyclable, paper, general_waste")
        sys.exit(1)

    base_yaw = load_base_yaw()
    wait_for_yaw()

    target_yaw = normalize_angle(
        base_yaw + math.radians(COMPARTMENT_OFFSETS_DEG[target_name])
    )

    old_state = get_current_compartment()
    print("Current saved compartment:", old_state)
    print("Target compartment:", target_name)
    print("Base yaw deg:", round(math.degrees(base_yaw), 2))
    print("Target yaw deg:", round(math.degrees(target_yaw), 2))

    pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
    rospy.sleep(0.5)

    rate = rospy.Rate(20)
    settled_count = 0
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - start_time > TIMEOUT_SEC:
            pub.publish(Twist())
            print("ERROR: Rotation timeout.")
            sys.exit(1)

        error = normalize_angle(target_yaw - current_yaw)
        error_deg = math.degrees(error)

        if abs(error_deg) <= TOL_DEG:
            settled_count += 1
        else:
            settled_count = 0

        if settled_count >= SETTLED_COUNT_REQUIRED:
            break

        cmd = Twist()
        w = KP * error
        if w > MAX_W:
            w = MAX_W
        if w < -MAX_W:
            w = -MAX_W
        cmd.angular.z = w
        pub.publish(cmd)

        print("yaw_error_deg:", round(error_deg, 2), "cmd_w:", round(w, 3))
        rate.sleep()

    pub.publish(Twist())
    rospy.sleep(0.3)
    pub.publish(Twist())

    save_current_compartment(target_name)
    print("DONE: compartment is now facing the user:", target_name)


def print_status():
    base_yaw = load_base_yaw()
    wait_for_yaw()
    current_state = get_current_compartment()
    print("Saved compartment:", current_state)
    print("Base yaw deg:", round(math.degrees(base_yaw), 2))
    print("Current yaw deg:", round(math.degrees(current_yaw), 2))
    for name in ["recyclable", "paper", "general_waste"]:
        y = normalize_angle(base_yaw + math.radians(COMPARTMENT_OFFSETS_DEG[name]))
        e = normalize_angle(y - current_yaw)
        print(name, "target_yaw_deg:", round(math.degrees(y), 2),
              "error_from_now_deg:", round(math.degrees(e), 2))


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("python ~/adaptive_compartment_rotate.py calibrate")
        print("python ~/adaptive_compartment_rotate.py recyclable")
        print("python ~/adaptive_compartment_rotate.py paper")
        print("python ~/adaptive_compartment_rotate.py general_waste")
        print("python ~/adaptive_compartment_rotate.py status")
        sys.exit(0)

    command = sys.argv[1].lower()

    rospy.init_node("adaptive_compartment_rotate")
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)

    if command == "calibrate":
        save_base_yaw()
    elif command == "status":
        print_status()
    else:
        rotate_to_compartment(command)


if __name__ == "__main__":
    main()
