#!/usr/bin/env python3
"""
Pillar/self-structure filter for the Kobuki bin platform.
Subscribes to /scan, removes the robot's own bin structure, republishes /scan_filtered.

The structure sits at a near-constant ~0.13-0.22 m radius around the laser at
several azimuths. A beam is nulled ONLY when its angle is in a structure band
AND its range is below SELF_CLEAR_M. Real walls/obstacles at those same angles
are always farther than this, so they pass through untouched -> localisation
(laser-on-walls) is preserved. The forward arc has no structure and is never
filtered, so close obstacles ahead are still seen.
"""
import math
import rospy
from sensor_msgs.msg import LaserScan

SELF_CLEAR_M = 0.30  # structure is < this; real walls are farther

# Azimuth bands (DEGREES, LASER FRAME) where the bin structure sits.
# Derived from scan_probe (~0.13-0.22 m returns across these clusters).
FILTER_BANDS_DEG = [
    (-142.0, -127.0),   # right pillar (orig -136..-130, widened)
    ( -72.0,  -59.0),   # strut
    ( -46.0,  -37.0),   # strut
    (  -3.0,   +5.0),   # rear-centre post (the dot on the TF origin)
    ( +41.0,  +50.0),   # strut
    ( +57.0,  +62.0),   # strut (~0.22 m)
    ( +64.0,  +80.0),   # strut (orig +70..+74, widened)
    (+126.0, +143.0),   # left pillar (orig +128..+138, widened)
]

FILTER_BANDS_RAD = [(math.radians(lo), math.radians(hi))
                    for (lo, hi) in FILTER_BANDS_DEG]


class PillarFilter(object):
    def __init__(self):
        self.pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.cb, queue_size=1)
        rospy.loginfo('pillar_filter: ready. SELF_CLEAR_M=%.2f bands(deg)=%s',
                      SELF_CLEAR_M, FILTER_BANDS_DEG)

    def cb(self, msg):
        new_ranges = list(msg.ranges)
        for i in range(len(new_ranges)):
            r = new_ranges[i]
            if not (r > 0.0 and r < SELF_CLEAR_M):
                continue  # leave real (far) returns and no-returns alone
            angle = msg.angle_min + i * msg.angle_increment
            for (lo, hi) in FILTER_BANDS_RAD:
                if lo <= angle <= hi:
                    new_ranges[i] = float('inf')
                    break
        msg.ranges = new_ranges
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('pillar_filter')
    PillarFilter()
    rospy.spin()
