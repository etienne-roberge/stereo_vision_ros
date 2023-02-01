import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.axes_grid1 import make_axes_locatable
import scipy, scipy.fftpack
import math
from sklearn.preprocessing import normalize

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

# load the input image and grab each channel -- note how OpenCV
# represents images as NumPy arrays with channels in Blue, Green,
# Red ordering rather than Red, Green, Blue
image = cv2.imread("../ExampleTouch/opaque/frame0008.jpg")
(B, G, R) = cv2.split(image)
# show each channel individually
#cv2.imshow("Red", R)
#cv2.imshow("Green", G)
#cv2.imshow("Blue", B)
cv2.imshow("RGB", image)

kernel_size = 3
kernel = np.array([[0,-1,-1],
				   [1,0,-1],
				   [1,1,0]],np.float32)

gB = cv2.filter2D(B, cv2.CV_64F, kernel)

kernel = np.array([[1,1,1],
				   [0,0,0],
				   [-1,-1,-1]],np.float32)

gR = cv2.filter2D(R, cv2.CV_64F, kernel)

kernel = np.array([[-1,0,1],
				   [-1,0,1],
				   [-1,0,1]],np.float32)

gG = cv2.filter2D(G, cv2.CV_64F, kernel)


testR = poisson_reconstruct(gR, gB, R.astype("float"))
ax = plt.subplot()
im = ax.imshow(testR)

divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="5%", pad=0.05)

plt.colorbar(im, cax=cax)

plt.show()


norm = np.linalg.norm(gR)
gR = gR/norm

norm = np.linalg.norm(gB)
gB = gB/norm
norm = np.linalg.norm(gG)
gG = gG/norm

# row_sums = gR.sum(axis=1)
# gRn = gR / row_sums[:, np.newaxis]
#
# row_sums = gG.sum(axis=1)
# gGn = gG / row_sums[:, np.newaxis]

total = (gG+gR+gB)/3.0

norm = np.linalg.norm(testR)
testR = testR/norm
#testR[testR < 0] = 0
total = testR

#gX = cv2.Laplacian(B,cv2.CV_64F)
ax = plt.subplot()
im = ax.imshow(total)

divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="5%", pad=0.05)

plt.colorbar(im, cax=cax)

plt.show()

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

# Make data.
X = np.arange(total.shape[1])
Y = np.arange(total.shape[0])
X, Y = np.meshgrid(X, Y)

# Plot the surface.
surf = ax.plot_surface(X, Y, total,cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)
ax.set_zlim(0, 0.15)

plt.show()



# gY = cv2.Sobel(R, cv2.CV_64F, 0, 1)
# testR = poisson_reconstruct(gX, gY, R.astype("float"))
# from matplotlib import cm
# # magnitude = np.sqrt((gX ** 2) + (gY ** 2))
# # orientation = np.arctan2(gY, gX) * (180 / np.pi) % 180
#
# ax = plt.subplot()
# im = ax.imshow(testR)
#
# divider = make_axes_locatable(ax)
# cax = divider.append_axes("right", size="5%", pad=0.05)
#
# plt.colorbar(im, cax=cax)
#
# plt.show()
#
# #######################
#
# gX = cv2.Sobel(B, cv2.CV_64F, 1, 0)
# gY = cv2.Sobel(B, cv2.CV_64F, 0, 1)
# testB = poisson_reconstruct(gX, gY, R.astype("float"))
#
# # magnitude = np.sqrt((gX ** 2) + (gY ** 2))
# # orientation = np.arctan2(gY, gX) * (180 / np.pi) % 180
#
# ax = plt.subplot()
# im = ax.imshow(testB)
#
# divider = make_axes_locatable(ax)
# cax = divider.append_axes("right", size="5%", pad=0.05)
#
# plt.colorbar(im, cax=cax)
#
# plt.show()
#
# #######################
#
# gX = cv2.Sobel(G, cv2.CV_64F, 1, 0)
# gY = cv2.Sobel(G, cv2.CV_64F, 0, 1)
# testG = poisson_reconstruct(gX, gY, R.astype("float"))
#
# # magnitude = np.sqrt((gX ** 2) + (gY ** 2))
# # orientation = np.arctan2(gY, gX) * (180 / np.pi) % 180
#
# ax = plt.subplot()
# im = ax.imshow(testG)
#
# divider = make_axes_locatable(ax)
# cax = divider.append_axes("right", size="5%", pad=0.05)
#
# plt.colorbar(im, cax=cax)
#
# plt.show()
#
#
# total = testG + testR + testB
#
# total[total < 0] = 0
#
# ax = plt.subplot()
# im = ax.imshow(total)
#
# divider = make_axes_locatable(ax)
# cax = divider.append_axes("right", size="5%", pad=0.05)
#
# plt.colorbar(im, cax=cax)
#
# plt.show()
#
# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
#
# # Make data.
# X = np.arange(total.shape[1])
# Y = np.arange(total.shape[0])
# X, Y = np.meshgrid(X, Y)
#
# # Plot the surface.
# surf = ax.plot_surface(X, Y, total,cmap=cm.coolwarm,
#                        linewidth=0, antialiased=False)
#
# # Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)
#
# plt.show()