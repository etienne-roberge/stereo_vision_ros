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

        self.pubDx_L = rospy.Publisher('/stereo_vision/dx_l', Image, queue_size=100)
        self.pubDy_L = rospy.Publisher('/stereo_vision/dy_l', Image, queue_size=100)
        self.pubDx_R = rospy.Publisher('/stereo_vision/dx_r', Image, queue_size=100)
        self.pubDy_R = rospy.Publisher('/stereo_vision/dy_r', Image, queue_size=100)

        rospy.init_node('DisparityGenerator', anonymous=True)
        rospy.Subscriber("/stereo_vision/left/image_rect", Image, self.callbackImageLeftRectified)
        rospy.Subscriber("/stereo_vision/right/image_rect", Image, self.callbackImageRightRectified)
        rospy.Subscriber("/stereo_vision/lights", String, self.callbackLights)


    def touchAnalyser(self):
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
                cv2.imshow("dxGAUCHE", self.dx)
                cv2.imshow("dyGAUCHE", self.dy)
                cv2.imshow("dxDROIT", self.dx_r)
                cv2.imshow("dyDROIT", self.dy_r)

                hsv = cv2.cvtColor(self.dx, cv2.COLOR_BGR2HSV)
                hsv2 = cv2.cvtColor(self.dy, cv2.COLOR_BGR2HSV)

                mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
                mask3 = cv2.inRange(hsv, lower_r, upper_r)
                mask4 = cv2.inRange(hsv, lower_r2, upper_r2)
                blue = cv2.bitwise_and(self.dx[:, :, 0], self.dx[:, :, 0], mask=mask2)
                red = cv2.bitwise_and(self.dx[:, :, 2], self.dx[:, :, 2], mask=mask3 + mask4)

                mask2 = cv2.inRange(hsv2, lower_blue, upper_blue)
                mask3 = cv2.inRange(hsv2, lower_r, upper_r)
                mask4 = cv2.inRange(hsv2, lower_r2, upper_r2)
                blueY = cv2.bitwise_and(self.dy[:, :, 0], self.dy[:, :, 0], mask=mask2)
                redY = cv2.bitwise_and(self.dy[:, :, 2], self.dy[:, :, 2], mask=mask3 + mask4)

                blue1 = blue.astype(float)
                red1 = red.astype(float)
                blue1[blue1 > 0] = 0.01
                red1[red1 > 0] = -0.01

                blueY1 = blueY.astype(float)
                redY1 = redY.astype(float)
                blueY1[blueY1 > 0] = 0.01
                redY1[redY1 > 0] = -0.01

                # blue1 = cv2.GaussianBlur(blue1, (5, 5), 0)
                # red1 = cv2.GaussianBlur(red1, (5, 5), 0)
                # blueY1 = cv2.GaussianBlur(blueY1, (5, 5), 0)
                # redY1 = cv2.GaussianBlur(redY1, (5, 5), 0)
                #
                # gxblue = cv2.Sobel(blue1, cv2.CV_16U, 0, 1, ksize=3).astype(float)
                # gxred = cv2.Sobel(red1, cv2.CV_16U, 0, 1, ksize=3).astype(float)
                # gyblue = cv2.Sobel(blueY1, cv2.CV_16U, 1, 0, ksize=3).astype(float)
                # gyred = cv2.Sobel(redY1, cv2.CV_16U, 1, 0, ksize=3).astype(float)

                # cv2.imshow("gxblue", gxblue)
                # cv2.imshow("gxred", gxred)
                # cv2.imshow("gyblue", gyblue)
                # cv2.imshow("gyred", gyred)

                dx = blue1 + red1
                dy = blueY1  + redY1

                # cv2.imshow("dxxx", dx)
                # cv2.imshow("dyyy", dy)

                testR1 = poisson_reconstruct(dy, dx, red.astype("float"))
                #testR1[testR1 <= 0] = 0


               # cv2.imshow("dxx", dx)
               # cv2.imshow("dyy", dy)

                #testRGAUCHE = poisson_reconstruct(dx, dy, red.astype("float"))
                testR1[testR1<=0] = 0
                cv2.imshow("norm_normal", testR1)
                filehandler = open("pickleTest_normal", 'wb')
                pickle.dump(testR1, filehandler)

#####

                # redY = redY * self.dy_red.T
                # blueY = blueY * self.dy_blue.T
                # blue = blue * self.dx_blue.T
                # red = red * self.dx_red.T

                blue1 = blue.astype(float)
                red1 = red.astype(float)
                #blue1 = blue1/-25500.0
                #red1 = red1/17700.0

                blueY1 = blueY.astype(float)
                redY1 = redY.astype(float)
                #blueY1 = blueY1/25500.0
                #redY1 = redY1/-13000.0

                # blue1 = cv2.GaussianBlur(blue1, (5, 5), 0)
                # red1 = cv2.GaussianBlur(red1, (5, 5), 0)
                # blueY1 = cv2.GaussianBlur(blueY1, (5, 5), 0)
                # redY1 = cv2.GaussianBlur(redY1, (5, 5), 0)
                #
                # gxblue = cv2.Sobel(blue1, cv2.CV_16U, 0, 1, ksize=3).astype(float)
                # gxred = cv2.Sobel(red1, cv2.CV_16U, 0, 1, ksize=3).astype(float)
                # gyblue = cv2.Sobel(blueY1, cv2.CV_16U, 1, 0, ksize=3).astype(float)
                # gyred = cv2.Sobel(redY1, cv2.CV_16U, 1, 0, ksize=3).astype(float)

                #cv2.imshow("gxblue", gxblue)
                #cv2.imshow("gxred", gxred)
                #cv2.imshow("gyblue", gyblue)
                #cv2.imshow("gyred", gyred)

                dx = blue1/-25500.0 + redY1/17700.0
                dy = blueY1/25500.0 + redY1/-13000.0

                #cv2.imshow("dxxx", dx)
                #cv2.imshow("dyyy", dy)

                testR1 = poisson_reconstruct(dx, dy, red.astype("float"))
                #testR1[testR1<=0] = 0
                testR1[testR1 <= 0] = 0
                #cv2.imshow("normalized_interpol", testR1)
                #filehandler = open("pickleTest_interpol", 'wb')
                #pickle.dump(testR1, filehandler)
####
                #
                # cv2.imshow("droiteX", self.dx_r)
                # cv2.imshow("droiteY", self.dy_r)
                #
                # hsv_droite = cv2.cvtColor(self.dx_r, cv2.COLOR_BGR2HSV)
                # hsv2_droite = cv2.cvtColor(self.dy_r, cv2.COLOR_BGR2HSV)
                #
                # mask2 = cv2.inRange(hsv_droite, lower_blue, upper_blue)
                # mask3 = cv2.inRange(hsv_droite, lower_r, upper_r)
                # mask4 = cv2.inRange(hsv_droite, lower_r2, upper_r2)
                # blue = cv2.bitwise_and(self.dx_r[:, :, 0], self.dx_r[:, :, 0], mask=mask2)
                # red = cv2.bitwise_and(self.dx_r[:, :, 2], self.dx[:, :, 2], mask=mask3 + mask4)
                #
                # mask2 = cv2.inRange(hsv2_droite, lower_blue, upper_blue)
                # mask3 = cv2.inRange(hsv2_droite, lower_r, upper_r)
                # mask4 = cv2.inRange(hsv2_droite, lower_r2, upper_r2)
                # blueY = cv2.bitwise_and(self.dy_r[:, :, 0], self.dy_r[:, :, 0], mask=mask2)
                # redY = cv2.bitwise_and(self.dy_r[:, :, 2], self.dy_r[:, :, 2], mask=mask3 + mask4)
                #
                # #print("bx:" + str(np.asarray(blue).max()) + " rx:" + str(np.asarray(red).max()) + " by:" + str(np.asarray(blueY).max()) + " ry:" + str(np.asarray(redY).max()))
                #
                # blue1 = blue.astype(float)
                # red1 = red.astype(float)
                # blue1[blue1 > 0] = -0.01
                # red1[red1 > 0] = 0.01
                #
                # blueY1 = blueY.astype(float)
                # redY1 = redY.astype(float)
                # blueY1[blueY1 > 0] = 0.01
                # redY1[redY1 > 0] = -0.01
                #
                # dx = blue1 + red1
                # dy = blueY1 + redY1
                #
                # # cv2.imshow("dxx", dx)
                # # cv2.imshow("dyy", dy)
                #
                # testRDROIT = poisson_reconstruct(dx, dy, red.astype("float"))
                #
                # cv2.imshow("rr_right", testRDROIT)
                #
                # #####
                # blue1 = blue.astype(float)
                # red1 = red.astype(float)
                # blue1 = blue1 / -25500.0
                # red1 = red1 / 12000.0
                #
                # blueY1 = blueY.astype(float)
                # redY1 = redY.astype(float)
                # blueY1 = blueY1 / 23000.0
                # redY1 = redY1 / -11000.0
                #
                # dx = blue1 + red1
                # dy = blueY1 + redY1
                #
                # # cv2.imshow("dxxx", dx)
                # # cv2.imshow("dyyy", dy)
                #
                # testR1 = poisson_reconstruct(dx, dy, red.astype("float"))
                # cv2.imshow("normalized_right", testR1)


####### COMBINATION!

                # hsv_combine = (hsv+hsv2)/2.0
                # hsv2_combine = (hsv+hsv2)/2.0
                # xx = (self.dx + self.dx_r) / 2.0
                # yy = (self.dy + self.dy_r) / 2.0
                #
                # cv2.imshow("xx", np.asarray(xx, dtype = 'uint8'))
                # cv2.imshow("yy", np.asarray(yy, dtype = 'uint8'))
                #
                # mask2 = cv2.inRange(hsv_combine, lower_blue, upper_blue)
                # mask3 = cv2.inRange(hsv_combine, lower_r, upper_r)
                # mask4 = cv2.inRange(hsv_combine, lower_r2, upper_r2)
                # blue = cv2.bitwise_and(xx[:, :, 0], xx[:, :, 0], mask=mask2)
                # red = cv2.bitwise_and(xx[:, :, 2], xx[:, :, 2], mask=mask3 + mask4)
                #
                # mask2 = cv2.inRange(hsv2_combine, lower_blue, upper_blue)
                # mask3 = cv2.inRange(hsv2_combine, lower_r, upper_r)
                # mask4 = cv2.inRange(hsv2_combine, lower_r2, upper_r2)
                # blueY = cv2.bitwise_and(yy[:, :, 0], yy[:, :, 0], mask=mask2)
                # redY = cv2.bitwise_and(yy[:, :, 2], yy[:, :, 2], mask=mask3 + mask4)
                #
                # # print("bx:" + str(np.asarray(blue).max()) + " rx:" + str(np.asarray(red).max()) + " by:" + str(np.asarray(blueY).max()) + " ry:" + str(np.asarray(redY).max()))
                #
                # blue1 = blue.astype(float)
                # red1 = red.astype(float)
                # blue1[blue1 > 0] = -0.03
                # red1[red1 > 0] = 0.03
                #
                # blueY1 = blueY.astype(float)
                # redY1 = redY.astype(float)
                # blueY1[blueY1 > 0] = 0.03
                # redY1[redY1 > 0] = -0.03
                #
                # dx = blue1 + red1
                # dy = blueY1 + redY1
                #
                # # cv2.imshow("dxx", dx)
                # # cv2.imshow("dyy", dy)

                #testR = poisson_reconstruct(dx, dy, red.astype("float"))

                #cv2.imshow("rr_combine", (testRGAUCHE + testRDROIT)/2)

                #####
                blue1 = blue.astype(float)
                red1 = red.astype(float)
                blue1 = blue1 / -25500.0
                red1 = red1 / 12000.0

                blueY1 = blueY.astype(float)
                redY1 = redY.astype(float)
                blueY1 = blueY1 / 23000.0
                redY1 = redY1 / -11000.0

                dx = blue1 + red1
                dy = blueY1 + redY1

                # cv2.imshow("dxxx", dx)
                # cv2.imshow("dyyy", dy)

                #testR1 = poisson_reconstruct(dx, dy, red.astype("float"))

                #cv2.imshow("normalized_combine", testR1)


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
            image_msg_R = self.bridge.cv2_to_imgmsg(self.dx_r, encoding="bgr8")
            self.pubDx_L.publish(image_msg)
            self.pubDx_R.publish(image_msg_R)
        else:
            self.dy = self.rectifiedLeft[:, 200:]
            self.dy_r = self.rectifiedRight[:, :-200]
            image_msg = self.bridge.cv2_to_imgmsg(self.dy, encoding="bgr8")
            image_msg2 = self.bridge.cv2_to_imgmsg(self.dy_r, encoding="bgr8")
            self.pubDy_L.publish(image_msg)
            self.pubDy_R.publish(image_msg2)
        self.lightsFlag = True

if __name__ == '__main__':
    dm = TouchGenerator()
    dm.touchAnalyserThread.start()
    rospy.spin()
    dm.running = False
