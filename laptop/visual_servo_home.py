#!/usr/bin/env python
# visual_servo_home.py
#
# Post-arrival visual servo controller for Capstone 2 Callable Sorting Garbage Bin.
#
# After move_base + fine_rotation_node land the robot approximately at HOME,
# this node engages visual feedback from the AprilTag to refine position and
# heading to within ~2 cm / ~2 deg of the tape-marked HOME pose, then publishes
# /initialpose to lock AMCL onto the true HOME coordinates.
#
# Architecture (analog of fine_rotation_node, but with camera feedback):
#   subscribe /tag_detections (AprilTag pose in optical frame)
#   load HOME target from ~/kobuki_apriltag/home_target_2026-05-10.yaml
#   P-control cmd_vel until current pose matches target within tolerance
#   on settled: stop, publish /initialpose to true HOME map coords, exit
#
# Sign conventions (per May 12 forward-push validation):
#   Robot drives forward => position.z DECREASES (camera approaches tag)
#     => v = +kp_lin * (current_z - target_z)
#     => if current_z > target_z (too far), v positive (drive forward, reduces z)
#
#   For angular: the May 6 verified-working sign is
#     w = +kp_ang * (target_x - current_x)
#   BUT this must be re-confirmed empirically on first live run. The May 3
#   work flipped this sign twice. If robot rotates AWAY from tag on first
#   trial, flip the sign in apply_control.

import math
import os
import sys
import yaml

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray


# ----- Tunables (start values - tune empirically) -----
KP_LIN = 0.5                # m/s per m of z-error
KP_ANG = 0.8                # rad/s per m of x-error
MAX_LIN = 0.10              # cap on linear velocity (m/s)
MAX_ANG = 0.6               # cap on angular velocity (rad/s)
MIN_LIN = 0.05              # below this, Kobuki won't move (preserve sign)
MIN_ANG = 0.15              # below this, Kobuki won't rotate (preserve sign)
TOL_X = 0.02                # 2 cm sideways tolerance
TOL_Z = 0.03                # 3 cm forward/back tolerance
RATE_HZ = 20                # control loop rate
SETTLED_THRESHOLD = 5       # consecutive in-tolerance frames before declaring done
WATCHDOG_SEC = 2.0          # if no detection for this long, stop and abort
TIMEOUT_SEC = 30.0          # overall give-up timer

# Target tag pose in optical frame (captured at HOME on May 12, sub-mm noise)
TARGET_YAML = os.path.expanduser('~/kobuki_apriltag/home_target_2026-05-10.yaml')

# HOME map-frame pose to publish on settle (from majlis_positions_v3.yaml)
HOME_MAP_X = -0.347
HOME_MAP_Y = -0.119
HOME_MAP_QZ = 0.996
HOME_MAP_QW = -0.087

CMD_VEL_TOPIC = '/mobile_base/commands/velocity'
TAG_TOPIC = '/tag_detections'
INITIALPOSE_TOPIC = '/initialpose'

# AprilTag ID we are servoing to (HOME tag)
HOME_TAG_ID = 0


def load_target(path):
    """Load target tag pose from YAML. Returns (target_x, target_z, tol_x, tol_z)."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    pos = data['home_tag_pose']['position']
    tol = data['tolerances']
    return float(pos['x']), float(pos['z']), float(tol['position_x']), float(tol['position_z'])


def clip_with_min(value, max_abs, min_abs):
    """Clip to [-max_abs, max_abs] and enforce min_abs floor (preserving sign).
       Returns 0 if input is exactly 0."""
    if value == 0.0:
        return 0.0
    if value > max_abs:
        return max_abs
    if value < -max_abs:
        return -max_abs
    if 0 < value < min_abs:
        return min_abs
    if -min_abs < value < 0:
        return -min_abs
    return value


class VisualServoHome(object):
    def __init__(self):
        rospy.init_node('visual_servo_home')

        # Load target from YAML (allow CLI override of tolerances if desired)
        try:
            tx, tz, tol_x_yaml, tol_z_yaml = load_target(TARGET_YAML)
        except Exception as e:
            rospy.logfatal('visual_servo_home: failed to load target YAML: %s', e)
            sys.exit(1)

        self.target_x = tx
        self.target_z = tz
        # Use YAML tolerances if present, else fall back to module constants
        self.tol_x = tol_x_yaml if tol_x_yaml else TOL_X
        self.tol_z = tol_z_yaml if tol_z_yaml else TOL_Z

        self.current_x = None
        self.current_z = None
        self.last_detection_time = None

        self.cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        self.initialpose_pub = rospy.Publisher(INITIALPOSE_TOPIC,
                                               PoseWithCovarianceStamped,
                                               queue_size=1, latch=True)

        rospy.Subscriber(TAG_TOPIC, AprilTagDetectionArray,
                         self.tag_cb, queue_size=1)

        rospy.loginfo('visual_servo_home: ready.')
        rospy.loginfo('  target: x=%.4f m (sideways), z=%.4f m (forward)',
                      self.target_x, self.target_z)
        rospy.loginfo('  tolerances: x=%.3f m, z=%.3f m',
                      self.tol_x, self.tol_z)
        rospy.loginfo('  gains: kp_lin=%.2f, kp_ang=%.2f', KP_LIN, KP_ANG)

        # Brief settle before engaging - wait for first detection
        rospy.loginfo('visual_servo_home: waiting for first detection...')
        t0 = rospy.Time.now()
        while self.current_x is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > 5.0:
                rospy.logfatal('visual_servo_home: no detection in 5 sec, aborting')
                sys.exit(1)
            rospy.sleep(0.1)

        rospy.loginfo('visual_servo_home: first detection: x=%.4f, z=%.4f',
                      self.current_x, self.current_z)

        self.run_control_loop()

    def tag_cb(self, msg):
        """Cache the most recent detection of the HOME tag."""
        for det in msg.detections:
            # det.id is a tuple/list; check first element
            if len(det.id) > 0 and det.id[0] == HOME_TAG_ID:
                self.current_x = det.pose.pose.pose.position.x
                self.current_z = det.pose.pose.pose.position.z
                self.last_detection_time = rospy.Time.now()
                return

    def run_control_loop(self):
        """Main P-control loop. Exits on settled, watchdog, or timeout."""
        rate = rospy.Rate(RATE_HZ)
        start_time = rospy.Time.now()
        settled_count = 0

        rospy.loginfo('visual_servo_home: ENGAGED')

        while not rospy.is_shutdown():
            # Overall timeout
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > TIMEOUT_SEC:
                rospy.logwarn('visual_servo_home: TIMEOUT after %.1f sec', elapsed)
                self.publish_stop()
                return

            # Watchdog: detection gone stale?
            if self.last_detection_time is not None:
                stale = (rospy.Time.now() - self.last_detection_time).to_sec()
                if stale > WATCHDOG_SEC:
                    rospy.logwarn('visual_servo_home: detection stale (%.1f s) - stopping',
                                  stale)
                    self.publish_stop()
                    return

            # Compute errors
            x_err = self.current_x - self.target_x   # sideways error (optical frame)
            z_err = self.current_z - self.target_z   # forward/back error (optical frame)

            # Done?
            if abs(x_err) < self.tol_x and abs(z_err) < self.tol_z:
                settled_count += 1
                rospy.loginfo('visual_servo_home: in-tol frame %d/%d (x_err=%.4f, z_err=%.4f)',
                              settled_count, SETTLED_THRESHOLD, x_err, z_err)
                if settled_count >= SETTLED_THRESHOLD:
                    self.publish_stop()
                    rospy.loginfo('visual_servo_home: SETTLED. '
                                  'final x_err=%.4f m, z_err=%.4f m, time=%.2f s',
                                  x_err, z_err, elapsed)
                    self.publish_initialpose()
                    return
                # Still need more in-tol frames; hold position
                self.publish_stop_once()
                rate.sleep()
                continue
            else:
                if settled_count > 0:
                    rospy.loginfo('visual_servo_home: settled_count reset '
                                  '(x_err=%.4f, z_err=%.4f)', x_err, z_err)
                settled_count = 0

            # Compute control commands
            # Linear: forward if too far (z too large), backward if too close
            v = KP_LIN * z_err
            v = clip_with_min(v, MAX_LIN, MIN_LIN)

            # Angular: rotate to center tag sideways
            # May 6 verified sign: w = +kp_ang * (target_x - current_x) = -kp_ang * x_err
            # If this rotates AWAY from tag on first trial, flip to: w = +kp_ang * x_err
            w = -KP_ANG * x_err
            w = clip_with_min(w, MAX_ANG, MIN_ANG)

            # Publish
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            # Throttled status log (every ~1 sec)
            if int(elapsed * RATE_HZ) % RATE_HZ == 0:
                rospy.loginfo('visual_servo_home: x_err=%+.4f, z_err=%+.4f, v=%+.3f, w=%+.3f',
                              x_err, z_err, v, w)

            rate.sleep()

    def publish_stop(self):
        """Send zero-velocity, repeated, to ensure Kobuki halts."""
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.05)

    def publish_stop_once(self):
        """Single zero-velocity publish (used while settling, to hold position)."""
        self.cmd_pub.publish(Twist())

    def publish_initialpose(self):
        """Re-seed AMCL with the true HOME map-frame pose."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = HOME_MAP_X
        msg.pose.pose.position.y = HOME_MAP_Y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = HOME_MAP_QZ
        msg.pose.pose.orientation.w = HOME_MAP_QW
        # Low covariance - we trust the visual servo landing
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.01    # x
        msg.pose.covariance[7] = 0.01    # y
        msg.pose.covariance[35] = 0.005  # yaw
        self.initialpose_pub.publish(msg)
        rospy.loginfo('visual_servo_home: published /initialpose to HOME map coords '
                      '(x=%.3f, y=%.3f)', HOME_MAP_X, HOME_MAP_Y)
        # Wait for AMCL to receive and apply the pose
        rospy.sleep(1.0)
        # Re-publish to be safe (in case first publish raced with subscription)
        self.initialpose_pub.publish(msg)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        VisualServoHome()
    except rospy.ROSInterruptException:
        pass
