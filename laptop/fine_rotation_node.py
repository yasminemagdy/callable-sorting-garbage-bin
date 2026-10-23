#!/usr/bin/env python
# fine_rotation_node.py
#
# Post-arrival fine rotation controller for Capstone 2 Callable Sorting Garbage Bin.
# Bypasses DWA's noisy goal-rotation by taking over after move_base declares
# "Goal reached" and rotating in place using direct AMCL pose feedback until
# yaw error is within tolerance.

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult


# ----- Tunables -----
THRESHOLD_DEG = 3.0
KP = 1.5
MAX_OMEGA = 0.6
MIN_OMEGA = 0.15
RATE_HZ = 20
TIMEOUT_SEC = 20.0
COVARIANCE_YAW_LIMIT = 0.05
CMD_VEL_TOPIC = '/mobile_base/commands/velocity'


def quat_to_yaw(qz, qw):
    return 2.0 * math.atan2(qz, qw)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FineRotationNode(object):
    def __init__(self):
        rospy.init_node('fine_rotation_node')

        self.target_yaw = None
        self.current_yaw = None
        self.current_yaw_cov = 1.0

        self.cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

        rospy.Subscriber('/move_base/current_goal', PoseStamped,
                         self.goal_cb, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.amcl_cb, queue_size=1)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult,
                         self.result_cb, queue_size=1)

        rospy.loginfo('fine_rotation_node: ready. threshold=%.1f deg, Kp=%.2f',
                      THRESHOLD_DEG, KP)
        rospy.spin()

    def goal_cb(self, msg):
        yaw = quat_to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        self.target_yaw = yaw
        rospy.loginfo('fine_rotation_node: new goal target yaw = %.1f deg',
                      math.degrees(yaw))

    def amcl_cb(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_yaw = quat_to_yaw(qz, qw)
        self.current_yaw_cov = msg.pose.covariance[35]

    def result_cb(self, msg):
        if msg.status.status != 3:
            rospy.logwarn('fine_rotation_node: move_base did not SUCCEED '
                          '(status=%d), skipping fine rotation',
                          msg.status.status)
            return

        if self.target_yaw is None:
            rospy.logwarn('fine_rotation_node: no goal yaw known, skipping')
            return

        if self.current_yaw is None:
            rospy.logwarn('fine_rotation_node: no AMCL pose yet, skipping')
            return

        if self.current_yaw_cov > COVARIANCE_YAW_LIMIT:
            rospy.logwarn('fine_rotation_node: AMCL yaw covariance %.4f > %.4f, '
                          'refusing to rotate',
                          self.current_yaw_cov, COVARIANCE_YAW_LIMIT)
            return

        self.execute_fine_rotation()

    def execute_fine_rotation(self):
        rospy.loginfo('fine_rotation_node: ENGAGED. target=%.1f deg, '
                      'current=%.1f deg, error=%.1f deg',
                      math.degrees(self.target_yaw),
                      math.degrees(self.current_yaw),
                      math.degrees(normalize_angle(self.target_yaw - self.current_yaw)))

        rate = rospy.Rate(RATE_HZ)
        threshold_rad = math.radians(THRESHOLD_DEG)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > TIMEOUT_SEC:
                rospy.logwarn('fine_rotation_node: timeout after %.1f sec', elapsed)
                self.publish_stop()
                return

            error = normalize_angle(self.target_yaw - self.current_yaw)

            if abs(error) < threshold_rad:
                self.publish_stop()
                rospy.loginfo('fine_rotation_node: SETTLED. final error=%.2f deg, '
                              'time=%.2f sec',
                              math.degrees(error), elapsed)
                return

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
            rate.sleep()

    def publish_stop(self):
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        FineRotationNode()
    except rospy.ROSInterruptException:
        pass
