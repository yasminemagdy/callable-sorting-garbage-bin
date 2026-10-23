#!/usr/bin/env python
# visual_servo_home.py  (hardened: angular/linear deadband + reacquire watchdog)
import math
import os
import sys
import yaml

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray

KP_LIN = 0.5
KP_ANG = 0.8
MAX_LIN = 0.10
MAX_ANG = 0.6
MIN_LIN = 0.05
MIN_ANG = 0.15
TOL_X = 0.02
TOL_Z = 0.03
RATE_HZ = 20
SETTLED_THRESHOLD = 5
WATCHDOG_SEC = 2.0          # stop moving if no detection this long
REACQUIRE_GIVEUP_SEC = 8.0  # only abort if still lost this long
TIMEOUT_SEC = 45.0

TARGET_YAML = os.path.expanduser('~/kobuki_apriltag/home_target_2026-05-10.yaml')

HOME_MAP_X = 0.130063062364
HOME_MAP_Y = -0.0399486733446
HOME_MAP_QZ = -0.998949108472
HOME_MAP_QW = 0.0458331613886

CMD_VEL_TOPIC = '/mobile_base/commands/velocity'
TAG_TOPIC = '/tag_detections'
INITIALPOSE_TOPIC = '/initialpose'
HOME_TAG_ID = 0


def load_target(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    pos = data['home_tag_pose']['position']
    tol = data['tolerances']
    return float(pos['x']), float(pos['z']), float(tol['position_x']), float(tol['position_z'])


def clip_with_min(value, max_abs, min_abs):
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
        try:
            tx, tz, tol_x_yaml, tol_z_yaml = load_target(TARGET_YAML)
        except Exception as e:
            rospy.logfatal('visual_servo_home: failed to load target YAML: %s', e)
            sys.exit(1)

        self.target_x = tx
        self.target_z = tz
        self.tol_x = tol_x_yaml if tol_x_yaml else TOL_X
        self.tol_z = tol_z_yaml if tol_z_yaml else TOL_Z

        self.current_x = None
        self.current_z = None
        self.last_detection_time = None

        self.cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        self.initialpose_pub = rospy.Publisher(INITIALPOSE_TOPIC,
                                               PoseWithCovarianceStamped,
                                               queue_size=1, latch=True)
        rospy.Subscriber(TAG_TOPIC, AprilTagDetectionArray, self.tag_cb, queue_size=1)

        rospy.loginfo('visual_servo_home: ready.')
        rospy.loginfo('  target: x=%.4f m (sideways), z=%.4f m (forward)',
                      self.target_x, self.target_z)
        rospy.loginfo('  tolerances: x=%.3f m, z=%.3f m', self.tol_x, self.tol_z)
        rospy.loginfo('  gains: kp_lin=%.2f, kp_ang=%.2f', KP_LIN, KP_ANG)

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
        for det in msg.detections:
            if len(det.id) > 0 and det.id[0] == HOME_TAG_ID:
                self.current_x = det.pose.pose.pose.position.x
                self.current_z = det.pose.pose.pose.position.z
                self.last_detection_time = rospy.Time.now()
                return

    def run_control_loop(self):
        rate = rospy.Rate(RATE_HZ)
        start_time = rospy.Time.now()
        settled_count = 0
        rospy.loginfo('visual_servo_home: ENGAGED')

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > TIMEOUT_SEC:
                rospy.logwarn('visual_servo_home: TIMEOUT after %.1f sec', elapsed)
                self.publish_stop()
                return

            # Detection freshness
            if self.last_detection_time is not None:
                stale = (rospy.Time.now() - self.last_detection_time).to_sec()
                if stale > REACQUIRE_GIVEUP_SEC:
                    rospy.logwarn('visual_servo_home: tag lost > %.1f s - giving up',
                                  REACQUIRE_GIVEUP_SEC)
                    self.publish_stop()
                    return
                if stale > WATCHDOG_SEC:
                    rospy.logwarn_throttle(1.0,
                        'visual_servo_home: tag lost - holding still to reacquire...')
                    self.publish_stop_once()
                    rate.sleep()
                    continue

            x_err = self.current_x - self.target_x
            z_err = self.current_z - self.target_z

            # Done?
            if abs(x_err) < self.tol_x and abs(z_err) < self.tol_z:
                settled_count += 1
                rospy.loginfo('visual_servo_home: in-tol frame %d/%d (x_err=%.4f, z_err=%.4f)',
                              settled_count, SETTLED_THRESHOLD, x_err, z_err)
                if settled_count >= SETTLED_THRESHOLD:
                    self.publish_stop()
                    rospy.loginfo('visual_servo_home: SETTLED. final x_err=%.4f m, '
                                  'z_err=%.4f m, time=%.2f s', x_err, z_err, elapsed)
                    self.publish_initialpose()
                    return
                self.publish_stop_once()
                rate.sleep()
                continue
            else:
                if settled_count > 0:
                    rospy.loginfo('visual_servo_home: settled_count reset')
                settled_count = 0

            # Control WITH DEADBANDS: no rotation if already centered,
            # no creep if already at target distance. This stops the
            # spurious MIN_ANG spin that swung the tag out of view.
            if abs(z_err) < self.tol_z:
                v = 0.0
            else:
                v = clip_with_min(KP_LIN * z_err, MAX_LIN, MIN_LIN)

            if abs(x_err) < self.tol_x:
                w = 0.0
            else:
                w = clip_with_min(-KP_ANG * x_err, MAX_ANG, MIN_ANG)

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            if int(elapsed * RATE_HZ) % RATE_HZ == 0:
                rospy.loginfo('visual_servo_home: x_err=%+.4f, z_err=%+.4f, v=%+.3f, w=%+.3f',
                              x_err, z_err, v, w)
            rate.sleep()

    def publish_stop(self):
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.05)

    def publish_stop_once(self):
        self.cmd_pub.publish(Twist())

    def publish_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = HOME_MAP_X
        msg.pose.pose.position.y = HOME_MAP_Y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = HOME_MAP_QZ
        msg.pose.pose.orientation.w = HOME_MAP_QW
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.005
        self.initialpose_pub.publish(msg)
        rospy.loginfo('visual_servo_home: published /initialpose to HOME (x=%.3f, y=%.3f)',
                      HOME_MAP_X, HOME_MAP_Y)
        rospy.sleep(1.0)
        self.initialpose_pub.publish(msg)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        VisualServoHome()
    except rospy.ROSInterruptException:
        pass
