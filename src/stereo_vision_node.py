#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import CameraInfo, Image
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge, CvBridgeError
import time
import threading

import tf

from arducam_manager import ArducamManager
from led_manager import LEDManager
from stereo_vision_ros.srv import ChangeModality, ChangeModalityResponse

CAMERA_NAME = 'stereo_vision_camera'

class StereoVisionNode:

    def __init__(self):
        # fps related variable
        self.timeFpsStart = time.time()
        self.timeFpsNow = time.time()
        self.seqFps = 0

        self.flagLowFPS = False

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

        self.service_change_modality = rospy.Service('change_modality', ChangeModality, self.change_modality)

        self.seq = 0
        self.stereo_cam.startCapture()

        self.checkCameraStatusThread = threading.Thread(target=self.checkCameraStatus)
        self.checkCameraStatusThreadRunning = True
        self.checkCameraStatusThread.start()

        self.ledManager = LEDManager()

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
            #transformation bidon
            br.sendTransform((5.0, 5.0, 5.0),
                             (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),
                             "stereo_vision_frame",
                             "world")
            self.seq += 1

    def publishImage(self, img_publisher, info_publisher, image, info, seq, timeNow):
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

        # Add timestamp and sequence number (empty by default)
        image_msg.header.stamp = timeNow
        image_msg.header.seq = seq
        image_msg.header.frame_id = "stereo_vision_frame"
        img_publisher.publish(image_msg)

        camera_msg = info
        camera_msg.header = image_msg.header  # Copy header from image message
        info_publisher.publish(camera_msg)

    def checkCameraStatus(self):
        while self.checkCameraStatusThreadRunning:
            self.timeFpsNow = time.time()
            if self.timeFpsNow - self.timeFpsStart >= 3:  # print every second
                fps = (self.seq - self.seqFps) / 3
                if fps == 0:
                    rospy.logerr("No data received from camera!")
                elif fps < 5:
                    rospy.logwarn("FPS is lower than 5...")
                    if not self.flagLowFPS: self.flagLowFPS = True
                elif self.flagLowFPS:
                    rospy.loginfo("FPS back to normal")
                    self.flagLowFPS = False

                self.seqFps = self.seq
                self.timeFpsStart = self.timeFpsNow

        self.checkCameraStatusThreadRunning = False

    def closeNode(self):
        self.checkCameraStatusThreadRunning = False
        self.checkCameraStatusThread.join()
        self.stereo_cam.shutdown()
        self.ledManager.shutdown()

    def startTouch(self):
        self.stereo_cam.startTouch()
        self.ledManager.startTouch()

    def startVision(self):
        self.stereo_cam.startVision()
        self.ledManager.startVision()

    def change_modality(self, request):
        result = True
        if request.modality == "touch":
            rospy.loginfo("Modality changed to TOUCH")
            self.startTouch()
        elif request.modality == "vision":
            rospy.loginfo("Modality changed to VISION")
            self.startVision()
        else:
            rospy.logwarn("Modality change error: " + request.modality + " is not recognised" )
            result = False
        return ChangeModalityResponse(result)

def main():
    stereoNode = StereoVisionNode()

    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        stereoNode.readImages(br)

    stereoNode.closeNode()


if __name__ == '__main__':
    main()
