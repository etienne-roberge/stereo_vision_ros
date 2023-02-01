import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.axes_grid1 import make_axes_locatable
import scipy, scipy.fftpack
import math
from sklearn.preprocessing import normalize


# load the input image and grab each channel -- note how OpenCV
# represents images as NumPy arrays with channels in Blue, Green,
# Red ordering rather than Red, Green, Blue
image = cv2.imread("../ExampleTouch/lighter/frame0002.jpg")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# define range of blue color in HSV
lower_blue = np.array([95, 10, 10]) 
upper_blue = np.array([135, 255, 255])
lower_g = np.array([35, 40, 40])
upper_g = np.array([80, 255, 255])
lower_r = np.array([170, 10, 10])
upper_r = np.array([255, 255, 255])
lower_r2 = np.array([0, 10, 10])
upper_r2 = np.array([15, 255, 255])
# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, lower_g, upper_g)
mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
mask3 = cv2.inRange(hsv, lower_r, upper_r)
mask4 = cv2.inRange(hsv, lower_r2, upper_r2)
# Bitwise-AND mask and original image
res = cv2.bitwise_and(image, image, mask=mask+mask2+mask3+mask4)
cv2.imshow('frame', image)
#cv2.imshow('mask', mask)
cv2.imshow('res', res)
cv2.waitKey()