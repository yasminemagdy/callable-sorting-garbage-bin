#!/usr/bin/env python3
import math, rospy
from sensor_msgs.msg import LaserScan
seen = {}
def cb(msg):
    for i, r in enumerate(msg.ranges):
        if 0.05 < r < 0.40:
            a = int(round(math.degrees(msg.angle_min + i*msg.angle_increment)))
            seen[a] = round(r, 3)
def shutdown():
    print("\nShort returns (<0.40 m) still in /scan_filtered, laser frame:")
    for a in sorted(seen):
        print("  angle=%4d deg   range=%.3f m" % (a, seen[a]))
rospy.init_node("scan_probe")
rospy.Subscriber("/scan_filtered", LaserScan, cb)
rospy.on_shutdown(shutdown)
rospy.loginfo("scan_probe: collecting ~12 s, keep the robot still...")
rospy.sleep(12.0)
rospy.signal_shutdown("done")
