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

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	bounding_box = ((0,0),(0,0))

	########## YOUR CODE STARTS HERE ##########

    # Find min and max values for each channel (H, S, V)
	# min_hsv = np.min(template_image_hsv, axis=(0, 1))  # Min per channel
	# max_hsv = np.max(template_image_hsv, axis=(0, 1))  # Max per channel

	# print(min_hsv)
	# print(max_hsv)

	# Using cv2.cvtColor() method
	# Using cv2.COLOR_BGR2HSV color spac

	# conversion code
	bgr_image = img
	hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
	crop_mask = np.zeros(hsv_image.shape[:2], dtype="uint8")
	cv2.rectangle(crop_mask, (0,175), (640,275), 255, -1)
	hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=crop_mask)

	# Define the lower and upper bounds for the hue range (orange color)
	lower_orange = np.array([0, 100, 100])
	upper_orange = np.array([20, 255, 255])
	

	# Create a mask using cv2.inRange
	mask = cv2.inRange(hsv_image, lower_orange, upper_orange)
	kernel = np.ones((5, 5), np.uint8)
	img_dilation = cv2.dilate(mask, kernel, iterations=1)
	#image_print(img_dilation)
	# Apply the mask to the original image
	#result = cv2.bitwise_and(image, image, mask=mask)
	#orange_points = np.nonzero(result)

	# Threshold the image to get only orange colors
	#mask = cv2.inRange(image, lower_orange, upper_orange)

	# Find contours
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
	x1 = 0
	y1 = 0
	x2 = 0
	y2 = 0
	length = 0
	# Find the largest contour (assuming the cone is the largest orange object)
	if contours:
		length = len(contours)
		largest_contour = max(contours, key=cv2.contourArea)

		# Get bounding box
		x1, y1, dx, dy = cv2.boundingRect(largest_contour)

		# Draw the bounding box on the original image
		cv2.rectangle(bgr_image, (x1, y1), (x1 + dx, y1 + dy), (0, 255, 0), 2)

		x2 = x1 + dx
		y2 = y1 + dy

		bounding_box =  ((x1, y1), (x2, y2))

	#image_print(hsv_image)

	#Displaying the image 
	#cv2.imshow(window_name, image)

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box, length
