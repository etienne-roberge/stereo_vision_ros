#!/usr/bin/env python
import rospy
import rosbag, cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from sensor_msgs.msg import PointCloud2
from image_geometry import StereoCameraModel
from cv_bridge import CvBridge
import utils_stereovision as stereovision
import threading
import time
import open3d as o3d

from open3d_ros_helper import open3d_ros_helper as orh

def bgr_from_imgmsg(msg):
    return CvBridge().imgmsg_to_cv2(msg)

class DisparityGenerator():
    def __init__(self):

        self.rectifiedLeft = None
        self.rectifiedRight = None
        self.cameraInfoLeft = None
        self.cameraInfoRight = None
        self.disparity = None
        self.rectifiedLeftFlag = False
        self.rectifiedRightFlag = False
        self.cameraInfoLeftFlag = False
        self.cameraInfoRightFlag = False
        self.disparityFlag = False

        self.header = None

        self.matchers = stereovision.Matchers(verbose=False)

        self.pub = rospy.Publisher('disparity', Image, queue_size=100)
        self.pub_point = rospy.Publisher('points2', PointCloud2, queue_size=100)

        self.bridge = CvBridge()

        self.model = StereoCameraModel()

        self.running = True
        self.disparityPublisherThread = threading.Thread(target=self.disparityDepthPublisher)

        rospy.init_node('DisparityGenerator', anonymous=True)
        rospy.Subscriber("/stereo_vision/left/image_rect", Image, self.callbackImageLeftRectified)
        rospy.Subscriber("/stereo_vision/right/image_rect", Image, self.callbackImageRightRectified)
        rospy.Subscriber("/stereo_vision/left/camera_info", CameraInfo, self.callbackLeftCameraInfo)
        rospy.Subscriber("/stereo_vision/right/camera_info", CameraInfo, self.callbackRightCameraInfo)


    def disparityDepthPublisher(self):
        while self.running:
            if self.rectifiedLeftFlag and self.rectifiedRightFlag and self.cameraInfoLeftFlag and self.cameraInfoRightFlag:

                disparity_map = self.matchers.computeFilteredDisparityMap(self.rectifiedLeft, self.rectifiedRight)
                self.model.fromCameraInfo(left_msg=self.cameraInfoLeft, right_msg=self.cameraInfoRight)
                points = cv2.reprojectImageTo3D(disparity_map, self.model.Q)
                rgb = cv2.cvtColor(self.rectifiedLeft, cv2.COLOR_BGR2RGB)

                mask = disparity_map > disparity_map.min()
                out_points = points[mask]
                out_colors = rgb[mask]
                #test open3d
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(out_points)
                norm_colors = out_colors / 255.0
                pcd.colors = o3d.utility.Vector3dVector(norm_colors)

                points = np.asarray(pcd.points)
                pcd_sel = pcd.select_by_index(np.where(points[:, 2] < 0.5)[0])

                #blur_pcd = pcd_sel.voxel_down_sample(voxel_size=0.001)
                print('[INFO] Point cloud blurred')
                filtered_pcd, ind1 = pcd_sel.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)
                print('[INFO] Point cloud filtered')
                #double_filtered_pcd, ind2 = filtered_pcd.remove_radius_outlier(nb_points=8, radius=0.03)
                #print('[INFO] Point cloud double filtered')
                #final_pcd = stereovision.sphere_crop_ply(double_filtered_pcd, radius=500)

                rospc = orh.o3dpc_to_rospc(filtered_pcd)
                rospc.header.frame_id = "map"
                rospc.header.stamp = rospy.Time.now()
                self.pub_point.publish(rospc)

                self.rectifiedLeftFlag = False
                self.rectifiedRightFlag = False
                self.cameraInfoLeftFlag = False
                self.cameraInfoRightFlag = False
            else:
                time.sleep(0.1)


    def callbackImageLeftRectified(self, data):
        self.header = data.header
        self.rectifiedLeft = bgr_from_imgmsg(data)
        self.rectifiedLeftFlag = True

    def callbackImageRightRectified(self, data):
        self.rectifiedRight = bgr_from_imgmsg(data)
        self.rectifiedRightFlag = True

    def callbackLeftCameraInfo(self, data):
        self.cameraInfoLeft = data
        self.cameraInfoLeftFlag = True

    def callbackRightCameraInfo(self, data):
        self.cameraInfoRight = data
        self.cameraInfoRightFlag = True


if __name__ == '__main__':
    dm = DisparityGenerator()
    dm.disparityPublisherThread.start()
    rospy.spin()
    dm.running = False
