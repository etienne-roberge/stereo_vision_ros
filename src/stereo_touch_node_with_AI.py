#!/usr/bin/env python
import rospy
import rosbag, cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge
import threading
import time
from std_msgs.msg import String
import scipy, scipy.fftpack
import math
import pickle
from tensorflow.keras.models import load_model

def bgr_from_imgmsg(msg):
    return CvBridge().imgmsg_to_cv2(msg)

def poisson_reconstruct(grady, gradx, boundarysrc):
	# Thanks to Dr. Ramesh Raskar for providing the original matlab code from which this is derived
	# Dr. Raskar's version is available here: http://web.media.mit.edu/~raskar/photo/code.pdf

	# Laplacian
	gyy = grady[1:,:-1] - grady[:-1,:-1]
	gxx = gradx[:-1,1:] - gradx[:-1,:-1]
	f = np.zeros(boundarysrc.shape)
	f[:-1,1:] += gxx
	f[1:,:-1] += gyy

	# Boundary image
	boundary = boundarysrc.copy()
	boundary[1:-1,1:-1] = 0

	# Subtract boundary contribution
	f_bp = -4*boundary[1:-1,1:-1] + boundary[1:-1,2:] + boundary[1:-1,0:-2] + boundary[2:,1:-1] + boundary[0:-2,1:-1]
	f = f[1:-1,1:-1] - f_bp

	# Discrete Sine Transform
	tt = scipy.fftpack.dst(f, norm='ortho')
	fsin = scipy.fftpack.dst(tt.T, norm='ortho').T

	# Eigenvalues
	(x,y) = np.meshgrid(range(1,f.shape[1]+1), range(1,f.shape[0]+1), copy=True)
	denom = (2*np.cos(math.pi*x/(f.shape[1]+2))-2) + (2*np.cos(math.pi*y/(f.shape[0]+2)) - 2)

	f = fsin/denom

	# Inverse Discrete Sine Transform
	tt = scipy.fftpack.idst(f, norm='ortho')
	img_tt = scipy.fftpack.idst(tt.T, norm='ortho').T

	# New center + old boundary
	result = boundary
	result[1:-1,1:-1] = img_tt

	return result

class TouchGenerator():
    def __init__(self):

        self.rectifiedLeft = None
        self.rectifiedRight = None
        self.rectifiedLeftFlag = False
        self.rectifiedRightFlag = False

        self.bridge = CvBridge()

        self.running = True
        self.touchAnalyserThread = threading.Thread(target=self.touchAnalyser)

        self.lights = "x"
        self.lightsFlag = False
        self.dx = None
        self.dx_r = None
        self.dy = None
        self.dy_r = None

        self.pubDx = rospy.Publisher('/stereo_vision/dx', Image, queue_size=100)
        self.pubDy = rospy.Publisher('/stereo_vision/dy', Image, queue_size=100)

        rospy.init_node('DisparityGenerator', anonymous=True)
        rospy.Subscriber("/stereo_vision/left/image_rect", Image, self.callbackImageLeftRectified)
        rospy.Subscriber("/stereo_vision/right/image_rect", Image, self.callbackImageRightRectified)
        rospy.Subscriber("/stereo_vision/lights", String, self.callbackLights)


    def touchAnalyser(self):
        # Load the pre-trained network from the .h5 file
        modelY = load_model('../Experiments/TouchExperiment/Dataset_clear/testY.h5')
        modelX = load_model('../Experiments/TouchExperiment/Dataset_clear/testX.h5')
        # define range of blue color in HSV
        lower_blue = np.array([95, 30, 30])
        upper_blue = np.array([135, 255, 255])
        lower_r = np.array([140, 30, 30])
        upper_r = np.array([179, 255, 255])
        lower_r2 = np.array([0, 35, 35])
        upper_r2 = np.array([30, 255, 255])
        while self.running:
            if self.lightsFlag:
                if self.dx is None:
                    continue
                if self.dy is None:
                    continue
                image_dx = self.dx
                image_dy = self.dy
                height, width = image_dy.shape[:2]

                hsv = cv2.cvtColor(image_dx, cv2.COLOR_BGR2HSV)
                hsv2 = cv2.cvtColor(image_dy, cv2.COLOR_BGR2HSV)

                mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
                mask3 = cv2.inRange(hsv, lower_r, upper_r)
                mask4 = cv2.inRange(hsv, lower_r2, upper_r2)
                dx_masked = cv2.bitwise_and(image_dx, image_dx, mask=mask2 + mask3 + mask4)

                mask2 = cv2.inRange(hsv2, lower_blue, upper_blue)
                mask3 = cv2.inRange(hsv2, lower_r, upper_r)
                mask4 = cv2.inRange(hsv2, lower_r2, upper_r2)
                dy_masked = cv2.bitwise_and(image_dy, image_dy, mask=mask2 + mask3 + mask4)

                cv2.imshow("image_dx", image_dx)
                cv2.imshow("image_dy", image_dy)
                cv2.imshow("dx_masked", dx_masked)
                cv2.imshow("dy_masked", dy_masked)

                resultY = np.zeros((height, width), dtype=float)
                dy_input = []
                # Get the positions of the non-zero values
                row_indices, col_indices = np.nonzero(dy_masked[:, :, 0] + dy_masked[:, :, 2])
                for row, col in zip(row_indices, col_indices):
                    dy_input.append([col, row, dy_masked[row, col, 0], dy_masked[row, col, 2]])
                    # dy_data.append([col, row, blue_dy[row, col], red_dy[row, col], -angle])

                dy_input = np.array(dy_input)
                if len(dy_input) > 0:
                    dy_output = modelY.predict(dy_input)

                for i in np.arange(len(dy_input)):
                    resultY[dy_input[i, 1], dy_input[i, 0]] = math.radians(dy_output[i])

                resultX = np.zeros((height, width), dtype=float)
                dx_input = []
                # Get the positions of the non-zero values
                row_indices, col_indices = np.nonzero(dx_masked[:, :, 0] + dx_masked[:, :, 2])
                for row, col in zip(row_indices, col_indices):
                    dx_input.append([col, row, dx_masked[row, col, 0], dx_masked[row, col, 2]])
                    # dy_data.append([col, row, blue_dy[row, col], red_dy[row, col], -angle])

                dx_input = np.array(dx_input)
                if len(dx_input) > 0:
                    dx_output = modelX.predict(dx_input)

                for i in np.arange(len(dx_input)):
                    resultX[dx_input[i, 1], dx_input[i, 0]] = math.radians(dx_output[i])

                testR1 = poisson_reconstruct(resultY, resultX, resultX.astype("float"))
                testR1 *= 1.0 / 15.0
                testR1[testR1 < 0] = 0

                cv2.imshow("AI", testR1)
                filehandler = open("pickleTest_AI", 'wb')
                pickle.dump(testR1, filehandler)

                cv2.waitKey(1)
                self.lightsFlag = False
            else:
                time.sleep(0.01)


    def callbackImageLeftRectified(self, data):
        self.rectifiedLeft = bgr_from_imgmsg(data)
        self.rectifiedLeftFlag = True

    def callbackImageRightRectified(self, data):
        self.rectifiedRight = bgr_from_imgmsg(data)
        self.rectifiedRightFlag = True

    def callbackLights(self, data):
        self.lights = data.data
        if data.data == "x":
            self.dx = self.rectifiedLeft[:, 200:]
            self.dx_r = self.rectifiedRight[:, :-200]
            image_msg = self.bridge.cv2_to_imgmsg(self.dx, encoding="bgr8")
            self.pubDx.publish(image_msg)
        else:
            self.dy = self.rectifiedLeft[:, 200:]
            self.dy_r = self.rectifiedRight[:, :-200]
            image_msg = self.bridge.cv2_to_imgmsg(self.dy, encoding="bgr8")
            self.pubDy.publish(image_msg)
        self.lightsFlag = True

if __name__ == '__main__':
    dm = TouchGenerator()
    dm.touchAnalyserThread.start()
    rospy.spin()
    dm.running = False
