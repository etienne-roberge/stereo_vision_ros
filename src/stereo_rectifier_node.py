#!/usr/bin/env python
import rospy
import rosbag, cv2
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel


def bgr_from_imgmsg(msg):
    return CvBridge().imgmsg_to_cv2(msg)

class DepthManager():
    def __init__(self):
        self.cameraInfoRightFlag = False
        self.cameraInfoLeftFlag = False
        self.leftMapX = None
        self.leftMapY = None
        self.rightMapX = None
        self.rightMapY = None

        self.model_right = PinholeCameraModel()
        self.model_left = PinholeCameraModel()

        self.bridge = CvBridge()

        self.pubLeft = rospy.Publisher('/stereo_vision/left/image_rect', Image, queue_size=100)
        self.pubRight = rospy.Publisher('/stereo_vision/right/image_rect', Image, queue_size=100)

        rospy.init_node('rectifier', anonymous=True)
        rospy.Subscriber("/stereo_vision/left/camera_info", CameraInfo, self.callbackLeftCameraInfo)
        rospy.Subscriber("/stereo_vision/right/camera_info", CameraInfo, self.callbackRightCameraInfo)
        rospy.Subscriber("/stereo_vision/left/image_raw", Image, self.callbackLeft)
        rospy.Subscriber("/stereo_vision/right/image_raw", Image, self.callbackRight)


    def callbackLeft(self, data):
        if self.cameraInfoLeftFlag:
            image = bgr_from_imgmsg(data)

            rect = cv2.remap(image, self.leftMapX, self.leftMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
            image_msg = self.bridge.cv2_to_imgmsg(rect, encoding="bgr8")
            image_msg.header = data.header
            self.pubLeft.publish(image_msg)


    def callbackRight(self, data):
        if self.cameraInfoRightFlag:
            image = bgr_from_imgmsg(data)

            rect = cv2.remap(image, self.rightMapX, self.rightMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
            image_msg = self.bridge.cv2_to_imgmsg(rect, encoding="bgr8")
            image_msg.header = data.header
            self.pubRight.publish(image_msg)

    def callbackLeftCameraInfo(self, data):
        if not self.cameraInfoLeftFlag:
            self.model_left.fromCameraInfo(data)
            self.leftMapX, self.leftMapY = cv2.initUndistortRectifyMap(self.model_left.K,
                                                                         self.model_left.D,
                                                                         self.model_left.R,
                                                                         self.model_left.P,
                                                                         (self.model_left.width,
                                                                          self.model_left.height),
                                                                       cv2.CV_32FC1)
            self.cameraInfoLeftFlag = True

    def callbackRightCameraInfo(self, data):
        if not self.cameraInfoRightFlag:
            self.model_right.fromCameraInfo(data)
            self.rightMapX, self.rightMapY = cv2.initUndistortRectifyMap(self.model_right.K,
                                                                         self.model_right.D,
                                                                         self.model_right.R,
                                                                         self.model_right.P,
                                                                         (self.model_right.width,
                                                                          self.model_right.height),
                                                                         cv2.CV_32FC1)
            self.cameraInfoRightFlag = True


if __name__ == '__main__':
    dm = DepthManager()
    rospy.spin()
    dm.running = False
