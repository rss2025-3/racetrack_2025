import cv2
import numpy as np

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


# Will return a list of lines, image is BGR8
# Send a list

def compute_roi(mask):
	height, width = mask.shape
	roi_mask = np.zeros_like(mask)

	# Vertices ordered clockwise starting bottom-left
	roi_corners = np.array([[
		(0,           height),
		(width,       height),
		(int(width*0.95), int(height*0.40)),   # top-right
		(int(width*0.05), int(height*0.40))    # top-left
	]], dtype=np.int32)

	cv2.fillPoly(roi_mask, roi_corners, 255)

	return roi_mask

def line_segmentation(img):
	# Convert to HSV for better color segmentation of white
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	lower_white = np.array([0, 0, 200])
	upper_white = np.array([180, 50, 255])
	mask = cv2.inRange(hsv, lower_white, upper_white)

	roi_mask = compute_roi(mask)

	# Apply ROI mask to the thresholded image
	roi_result = cv2.bitwise_and(mask, roi_mask)

	# Use Canny edge detection on the ROI result.
	edges = cv2.Canny(roi_result, 50, 150)

	# Use Probabilistic Hough Transform to detect line segments.
	lines = cv2.HoughLinesP(edges, 
							rho=1, 
							theta=np.pi / 180, 
							threshold=50, 
							minLineLength=50, 
							maxLineGap=10)

	return lines
