#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo

class CameraInfoSyncer:
    def __init__(self):
        # Publish image and camera_info under same namespace with same timestamp
        self.image_pub = rospy.Publisher('/cam_synced/image_raw', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/cam_synced/camera_info', CameraInfo, queue_size=10)
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)

        # Approximate Orbbec intrinsics at 640x480
        self.K = [600.0, 0.0, 320.0,
                  0.0, 600.0, 240.0,
                  0.0, 0.0, 1.0]
        self.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        self.P = [600.0, 0.0, 320.0, 0.0,
                  0.0, 600.0, 240.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        rospy.loginfo("CameraInfoSyncer running.")

    def callback(self, img_msg):
        # Republish image as-is
        self.image_pub.publish(img_msg)

        # Build matching camera_info with identical header
        info = CameraInfo()
        info.header = img_msg.header
        info.height = 480
        info.width = 640
        info.distortion_model = 'plumb_bob'
        info.D = self.D
        info.K = self.K
        info.R = self.R
        info.P = self.P
        self.info_pub.publish(info)

if __name__ == '__main__':
    rospy.init_node('camera_info_syncer')
    CameraInfoSyncer()
    rospy.spin()
