#!/usr/bin/env python
# rotate_to_home_yaw.py
#
# Post-servo orientation corrector for Capstone 2 Callable Sorting Garbage Bin.
#
# After visual_servo_home.py settles the robot at the HOME tape mark facing
# the AprilTag, this node rotates the robot in place until its yaw matches
# the stored "operating HOME" yaw (facing the Majlis), using AMCL feedback.
#
# Architecture (analog of fine_rotation_node, but with HOME yaw hardcoded):
#   subscribe /amcl_pose
#   compute shortest-path yaw error to target_yaw
#   P-control cmd_vel until |yaw_error| < THRESHOLD_DEG
#   stop, exit

import math
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped


# ----- Target HOME yaw (operating orientation - facing Majlis) -----
# From ~/kobuki_nav_config/waypoints/majlis_positions_v3.yaml: yaw_deg: 9.98
TARGET_YAW_DEG = -5.56
TARGET_YAW_RAD = math.radians(TARGET_YAW_DEG)

# ----- Tunables -----
THRESHOLD_DEG = 2.0          # Stop when |yaw_error| below this
KP = 1.5                     # Proportional gain (rad/s per rad of error)
MAX_OMEGA = 0.6              # Max angular velocity (rad/s)
MIN_OMEGA = 0.15             # Min effective omega (Kobuki dead-zone)
RATE_HZ = 20                 # Control loop rate
TIMEOUT_SEC = 15.0           # Bail out if rotation takes too long
COVARIANCE_YAW_LIMIT = 0.05  # Refuse to rotate if AMCL yaw uncertain
CMD_VEL_TOPIC = '/mobile_base/commands/velocity'


def quat_to_yaw(qz, qw):
    """Convert quaternion (z, w) to yaw radians for ground robot."""
    return 2.0 * math.atan2(qz, qw)


def normalize_angle(a):
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RotateToHomeYaw(object):
    def __init__(self):
        rospy.init_node('rotate_to_home_yaw')

        self.current_yaw = None
        self.current_yaw_cov = 1.0

        self.cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.amcl_cb, queue_size=1)

        rospy.loginfo('rotate_to_home_yaw: ready. target=%.2f deg, threshold=%.1f deg, Kp=%.2f',
                      TARGET_YAW_DEG, THRESHOLD_DEG, KP)

        # Wait for first AMCL pose
        rospy.loginfo('rotate_to_home_yaw: waiting for AMCL pose...')
        t0 = rospy.Time.now()
        while self.current_yaw is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > 5.0:
                rospy.logfatal('rotate_to_home_yaw: no AMCL pose in 5 sec, aborting')
                return
            rospy.sleep(0.1)

        if self.current_yaw_cov > COVARIANCE_YAW_LIMIT:
            rospy.logwarn('rotate_to_home_yaw: AMCL yaw covariance %.4f > %.4f, '
                          'refusing to rotate (localization uncertain)',
                          self.current_yaw_cov, COVARIANCE_YAW_LIMIT)
            return

        initial_err = normalize_angle(TARGET_YAW_RAD - self.current_yaw)
        rospy.loginfo('rotate_to_home_yaw: initial yaw=%.2f deg, target=%.2f deg, '
                      'error=%.2f deg',
                      math.degrees(self.current_yaw),
                      TARGET_YAW_DEG,
                      math.degrees(initial_err))

        self.run_control_loop()

    def amcl_cb(self, msg):
        """Cache current yaw and covariance from AMCL."""
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = quat_to_yaw(qz, qw)
        # covariance is 6x6 row-major; yaw variance is index [35]
        self.current_yaw_cov = msg.pose.covariance[35]

    def run_control_loop(self):
        """The actual rotation control loop."""
        rate = rospy.Rate(RATE_HZ)
        threshold_rad = math.radians(THRESHOLD_DEG)
        start_time = rospy.Time.now()

        rospy.loginfo('rotate_to_home_yaw: ENGAGED')

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > TIMEOUT_SEC:
                rospy.logwarn('rotate_to_home_yaw: timeout after %.1f sec, stopping',
                              elapsed)
                self.publish_stop()
                return

            error = normalize_angle(TARGET_YAW_RAD - self.current_yaw)

            if abs(error) < threshold_rad:
                self.publish_stop()
                rospy.loginfo('rotate_to_home_yaw: SETTLED. final error=%.2f deg, '
                              'final yaw=%.2f deg, time=%.2f sec',
                              math.degrees(error),
                              math.degrees(self.current_yaw),
                              elapsed)
                return

            # P-control with clipping
            omega = KP * error
            if omega > MAX_OMEGA:
                omega = MAX_OMEGA
            elif omega < -MAX_OMEGA:
                omega = -MAX_OMEGA
            if 0 < omega < MIN_OMEGA:
                omega = MIN_OMEGA
            elif -MIN_OMEGA < omega < 0:
                omega = -MIN_OMEGA

            cmd = Twist()
            cmd.angular.z = omega
            self.cmd_pub.publish(cmd)

            # Throttled status log
            if int(elapsed * RATE_HZ) % RATE_HZ == 0:
                rospy.loginfo('rotate_to_home_yaw: yaw=%+.2f deg, err=%+.2f deg, omega=%+.3f',
                              math.degrees(self.current_yaw),
                              math.degrees(error),
                              omega)

            rate.sleep()

    def publish_stop(self):
        """Send zero-velocity, repeated, to ensure Kobuki stops."""
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        RotateToHomeYaw()
    except rospy.ROSInterruptException:
        pass
