#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

HOME_X = 0.00813767540314
HOME_Y = 0.0194611676551
# Majlis-facing (+9.98deg) operating start:
QZ, QW = -0.0484963978742, 0.998823357453
# Tag-facing (~-170deg): QZ, QW = 0.996, -0.087

COV = [0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
       0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
       0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
       0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
       0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
       0.0,  0.0,  0.0, 0.0, 0.0, 0.06853892326654787]

def main():
    rospy.init_node('home_localize', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(1.0)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = HOME_X
    msg.pose.pose.position.y = HOME_Y
    msg.pose.pose.orientation.z = QZ
    msg.pose.pose.orientation.w = QW
    msg.pose.covariance = COV
    rate = rospy.Rate(5)
    for _ in range(5):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    rospy.loginfo("HOME pose published (%.3f, %.3f)" % (HOME_X, HOME_Y))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
