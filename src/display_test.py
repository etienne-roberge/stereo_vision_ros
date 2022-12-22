#!/usr/bin/env python
import rospy
import rosbag, cv2
from sensor_msgs.msg import CompressedImage, Image
import sys

class ImageConversions():
    # We only instantiate the bridge once
    bridge = None
    
def get_cv_bridge():
    if ImageConversions.bridge is None:
        from cv_bridge import CvBridge  # @UnresolvedImport
        ImageConversions.bridge = CvBridge()
    return ImageConversions.bridge

def bgr_from_imgmsg(msg):
    bridge = get_cv_bridge()
    return bridge.imgmsg_to_cv2(msg)
    
def rgb_from_imgmsg(msg):
    bridge = get_cv_bridge()
    return bridge.imgmsg_to_cv2(msg, "rgb8")
    
def callback(data):
    cv2.namedWindow("ArduCam Demo",1)
    image = bgr_from_imgmsg(data)
    image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("ArduCam Demo",image)
    cv2.waitKey(10)
    
def listener(camera):
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/stereo_vision/"+camera+"/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    camera = "left"
    if len(sys.argv) ==  2:
        camera = str(sys.argv[1])
    listener(camera)
