#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import CameraInfo, Image
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge, CvBridgeError
import time

import tf

from arducam_manager import ArducamManager

CAMERA_NAME = 'sdfsdf'
DEFAULT_IMAGE_TOPIC = 'image_raw'
DEFAULT_CAMERA_TOPIC = 'camera_info'


class StereoVisionNode:

    def __init__(self):
        # fps related variable
        self.timeFpsStart = time.time()
        self.timeFpsNow = time.time()
        self.seqFps = 0

        rospy.init_node('{}_node'.format(CAMERA_NAME), argv=sys.argv)

        self.config_file = rospy.get_param('~config_file',
                                           '../camera_config_files/OV5647_MIPI_2Lane_RAW8_8b_2592x1944_AB.cfg')
        self.calibration_left = rospy.get_param('~calibration_left', '')
        self.calibration_right = rospy.get_param('~calibration_right', '')

        self.cim_left = CameraInfoManager(cname="left",
                                          url='file://' + self.calibration_left,
                                          namespace="left")
        self.cim_right = CameraInfoManager(cname="right",
                                           url='file://' + self.calibration_right,
                                           namespace="right")

        self.cim_left.loadCameraInfo()  # Needs to be called before getter!
        self.camera_info_left = self.cim_left.getCameraInfo()
        self.camera_info_left.header.frame_id = "left_frame"

        self.cim_right.loadCameraInfo()  # Needs to be called before getter!
        self.camera_info_right = self.cim_right.getCameraInfo()
        self.camera_info_right.header.frame_id = "right_frame"

        self.stereo_cam = ArducamManager(self.config_file)
        self.bridge = CvBridge()

        self.image_publisher_right = rospy.Publisher("right/image_raw", Image,
                                                     queue_size=1)
        self.image_publisher_left = rospy.Publisher("left/image_raw", Image,
                                                    queue_size=1)
        self.camera_publisher_right = rospy.Publisher("right/camera_info", CameraInfo,
                                                      queue_size=1)
        self.camera_publisher_left = rospy.Publisher("left/camera_info", CameraInfo,
                                                     queue_size=1)

        self.seq = 0
        self.stereo_cam.startCapture()

    def readImages(self, br):

        # Read image from camera
        img_left, img_right = self.stereo_cam.readImages()

        if img_left is not None and img_right is not None:
            timeNow = rospy.Time.now()
            self.publishImage(self.image_publisher_left,
                              self.camera_publisher_left,
                              img_left,
                              self.camera_info_left,
                              self.seq,
                              timeNow)

            self.publishImage(self.image_publisher_right,
                              self.camera_publisher_right,
                              img_right,
                              self.camera_info_right,
                              self.seq,
                              timeNow)
            br.sendTransform((5.0, 5.0, 5.0),
                             (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),
                             "camera_etienne",
                             "world")
            self.seq += 1
        self.printFps()

    def publishImage(self, img_publisher, info_publisher, image, info, seq, timeNow):
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

        # Add timestamp and sequence number (empty by default)
        image_msg.header.stamp = timeNow
        image_msg.header.seq = seq
        image_msg.header.frame_id = "camera_etienne"
        img_publisher.publish(image_msg)

        camera_msg = info
        camera_msg.header = image_msg.header  # Copy header from image message
        info_publisher.publish(camera_msg)


    def printFps(self):
        self.timeFpsNow = time.time()
        if self.timeFpsNow - self.timeFpsStart >= 1:  # print every second
            print("%s %d %s" % ("fps:", self.seq - self.seqFps, "/s"))
            self.seqFps = self.seq
            self.timeFpsStart = self.timeFpsNow

    def closeNode(self):
        self.stereo_cam.shutdown()


def main():
    stereoNode = StereoVisionNode()

    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        stereoNode.readImages(br)

    stereoNode.closeNode()


if __name__ == '__main__':
    main()