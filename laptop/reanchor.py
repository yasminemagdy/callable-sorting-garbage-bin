#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('reanchor', anonymous=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
rospy.loginfo('reanchor: waiting for current /amcl_pose...')
msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
out = PoseWithCovarianceStamped()
out.header.frame_id = 'map'
out.header.stamp = rospy.Time.now()
out.pose.pose = msg.pose.pose
cov = [0.0]*36
cov[0] = 0.05    # x variance
cov[7] = 0.05    # y variance
cov[35] = 0.02   # yaw variance
out.pose.covariance = cov
rospy.sleep(0.5)
pub.publish(out)
rospy.loginfo('reanchor: re-published current pose to /initialpose with tight covariance.')
rospy.sleep(1.0)
