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

PTS_IMAGE_PLANE = [[483, 210],
                   [244, 206],
                   [173, 249],
                   [584, 286]]

PTS_GROUND_PLANE = [[39, -13],
                    [39, 13],
                    [23, 13],
                    [23, -13]]

METERS_PER_INCH = 0.0254

np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np_pts_ground * METERS_PER_INCH
np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np_pts_image * 1.0
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

# To this:
h, _ = cv2.findHomography(np_pts_image, np_pts_ground)

def transformUvToXy(u, v):
    """
    u and v are pixel coordinates.
    The top left pixel is the origin, u axis increases to right, and v axis
    increases down.

    Returns a normal non-np 1x2 matrix of xy displacement vector from the
    camera to the point on the ground plane.
    Camera points along positive x axis and y axis increases to the left of
    the camera.

    Units are in meters.
    """
    homogeneous_point = np.array([[u], [v], [1]])
    xy = np.dot(h, homogeneous_point)
    scaling_factor = 1.0 / xy[2, 0]
    homogeneous_xy = xy * scaling_factor
    x = homogeneous_xy[0, 0]
    y = homogeneous_xy[1, 0]
    return x, y

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def point_to_line_distance(point, line_start, line_end):
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return np.linalg.norm(point_vec)
    
    # Calculate projection
    t = max(0, min(1, np.dot(point_vec, line_vec) / (line_len * line_len)))
    projection = line_start + t * line_vec
    return np.linalg.norm(point - projection)

def line_to_line_distance(line1, line2):
    p1 = np.array([line1[0], line1[1]])
    p2 = np.array([line1[2], line1[3]])
    p3 = np.array([line2[0], line2[1]])
    p4 = np.array([line2[2], line2[3]])
    
    distances = [
        point_to_line_distance(p1, p3, p4),
        point_to_line_distance(p2, p3, p4),
        point_to_line_distance(p3, p1, p2),
        point_to_line_distance(p4, p1, p2)
    ]
    return min(distances)
def select_lane_lines(regression_lines, angle_filter=-15):
    """
    Select the two lines that form the lane boundaries on either side of the vertical center line.
    Returns the left and right lane boundary lines, along with their homography transformed coordinates.
    """
    if len(regression_lines) < 2:
        return None, None, None, None
        
    # Transform lines to bird's eye view coordinates
    transformed_lines = []
    filtered_lines = []
    for line in regression_lines:
        x1, y1, x2, y2 = line

        # Transform both points
        p1_t = transformUvToXy(x1, y1)
        p2_t = transformUvToXy(x2, y2)
        
        # Ensure x1 is the minimum x coordinate
        if p2_t[0] < p1_t[0]:
            x1_t, y1_t = p2_t
            x2_t, y2_t = p1_t
        else:
            x1_t, y1_t = p1_t
            x2_t, y2_t = p2_t
        
        # Calculate angle after transform
        angle = np.arctan2(y2_t - y1_t, x2_t - x1_t) * 180 / np.pi

        if angle > angle_filter:
            transformed_lines.append(((x1_t, y1_t), (x2_t, y2_t)))
            filtered_lines.append(line)
    
    if len(filtered_lines) < 2:
        return None, None, None, None
    
    # Find lines on either side of center (x=0)
    left_lines = []
    right_lines = []
    
    for i, ((x1, y1), _) in enumerate(transformed_lines):
        print(f"{x1}, {y1}")
        if y1 < 0:
            left_lines.append((i, abs(y1)))
        else:
            right_lines.append((i, abs(y1)))
    
    left_idx = min(left_lines, key=lambda x: x[1])[0] if left_lines else None
    right_idx = min(right_lines, key=lambda x: x[1])[0] if right_lines else None
    
    left_line = filtered_lines[left_idx] if left_idx is not None else None
    right_line = filtered_lines[right_idx] if right_idx is not None else None
    
    left_transformed = transformed_lines[left_idx] if left_idx is not None else None
    right_transformed = transformed_lines[right_idx] if right_idx is not None else None
    
    return left_line, right_line, left_transformed, right_transformed

def cluster_lines(lines, distance_threshold=100, angle_threshold=10):
    if lines is None or len(lines) == 0:
        return [], []
    
    lines_array = np.array([line[0] for line in lines])
    
    # Calculate line angles
    angles = np.arctan2(lines_array[:, 3] - lines_array[:, 1], 
                       lines_array[:, 2] - lines_array[:, 0]) * 180 / np.pi
    angles = np.mod(angles + 180, 180)
    
    clusters = []
    used_lines = set()

    for i, line in enumerate(lines_array):
        if i in used_lines:
            continue
            
        current_cluster = [i]
        used_lines.add(i)
        
        # Compare with all other lines
        for j, other_line in enumerate(lines_array):
            if j in used_lines:
                continue
                
            # Check angle difference
            angle_diff = abs(angles[i] - angles[j])
            angle_diff = min(angle_diff, 180 - angle_diff)
            
            if angle_diff > angle_threshold:
                continue
                
            # Calculate minimum distance between lines
            distance = line_to_line_distance(line, other_line)
            
            if distance < distance_threshold:
                current_cluster.append(j)
                used_lines.add(j)
        
        clusters.append(current_cluster)
    
    final_lines = []
    regression_lines = []
    cluster_assignments = []
    
    for cluster_idx, cluster in enumerate(clusters):
        cluster_lines = lines_array[cluster]
        
        # Store cluster assignments
        for line_idx in cluster:
            cluster_assignments.append((line_idx, cluster_idx))
        
        # Average the endpoints for the cluster representation
        x1 = np.mean(cluster_lines[:, 0])
        y1 = np.mean(cluster_lines[:, 1])
        x2 = np.mean(cluster_lines[:, 2])
        y2 = np.mean(cluster_lines[:, 3])
        final_lines.append([int(x1), int(y1), int(x2), int(y2)])
        
        # Collect all points for regression
        points = np.vstack([
            cluster_lines[:, [0, 1]],  # Start points
            cluster_lines[:, [2, 3]]   # End points
        ])
        
        # Perform linear regression
        x = points[:, 0]
        y = points[:, 1]
        
        if len(x) > 1:  # Need at least 2 points for regression
            coeffs = np.polyfit(x, y, 1)
            
            # Find the extent of the line
            x_min = int(np.min(x))
            x_max = int(np.max(x))
            
            # Calculate corresponding y values
            y_min = int(coeffs[0] * x_min + coeffs[1])
            y_max = int(coeffs[0] * x_max + coeffs[1])
            
            regression_line = [x_min, y_min, x_max, y_max]
            regression_lines.append(regression_line)
    
    return np.array(final_lines), np.array(regression_lines), cluster_assignments

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
							minLineLength=100, 
							maxLineGap=10)

	if lines is not None:
		clustered_lines, regression_lines, cluster_assignments = cluster_lines(
			lines=lines,
			distance_threshold=100,
			angle_threshold=10,
			)
		
		# Get lane boundary lines
		left_line, right_line = select_lane_lines(regression_lines)

		return left_line, right_line
	else:
		None