import cv2
import numpy as np
import traceback
from dataclasses import dataclass, field
from typing import List

# Set in the form (width, height)
default_roi = np.float32([
		(550, 450),  # Top-left corner
		(160, 720),  # Bottom-left corner            
		(1330, 720), # Bottom-right corner
		(775, 450)   # Top-right corner
	])


@dataclass
class LaneLine:
	type: str = ''
	color: str = ''
	is_solid: bool = False
	curvature: float = 0.0
	points: List[np.ndarray] = field(default_factory=list)
	avg_fit: List[float] = field(default_factory=list)
	
@dataclass
class Lane:
	left: LaneLine = field(default_factory=LaneLine)
	right: LaneLine = field(default_factory=LaneLine)
	is_host: bool = False

class LaneDetector:
	def __init__(self): 
		width = 1280
		height = 720
		roi = default_roi

		self.set_constants(width, height)
		self.set_roi(roi)

		# Each image step along the way
		self.isolated_lane_lines = None
		self.cropped_lane_lines = None
		self.warped_frame = None
		self.histogram = None
		self.transformation_matrix = None
		self.inv_transformation_matrix = None
				 
		self.lane_lines = []
		self.fallback_right = np.array([0.0001018, -0.795, 40])
		self.fallback_left = np.array([-0.000122, -0.15, 28])

		# Best fit polynomial lines for left line and right line of the lane
		self.left_fit = None
		self.right_fit = None
		self.left_lane_inds = None
		self.right_lane_inds = None
		self.ploty = None
		self.left_fitx = None
		self.right_fitx = None
		self.leftx = None
		self.rightx = None
		self.lefty = None
		self.righty = None

		# Pixel parameters for x and y dimensions
		self.YM_PER_PIX = 10.0 / 1000 # meters per pixel in y dimension
		self.XM_PER_PIX = 3.7 / 781 # meters per pixel in x dimension
				 
		# Radii of curvature and offset
		self.left_curvem = None
		self.right_curvem = None
		self.center_offset = None
 
	def set_constants(self, width, height):
		self.width = width
		self.height = height

		self.padding = int(0.25 * width) # padding from side of the image in pixels

		self.desired_roi_points = np.float32([
			[self.padding, 0],              # Top-left corner
			[self.padding, height],       	# Bottom-left corner         
			[width - self.padding, height], # Bottom-right corner
			[width - self.padding, 0]      	# Top-right corner
		])

		# Sliding window parameters
		self.no_of_windows = 10
		self.margin = int((1/12) * width)  # Window width is +/- margin
		self.minpix = int((1/24) * width)  # Min no. of pixels to recenter window

	def set_roi(self, roi):
		self.roi_points = roi

	def set_roi_points(self, x, y):
		"""
		x: x-coordinates of the points [x1, x2, x3, x4]
		y: y-coordinates of the points [y1, y2, y3, y4]
		"""
		self.roi_points = np.float32([(x[0], y[0]), (x[1], y[1]), (x[2], y[2]), (x[3], y[3])])

	@staticmethod
	def binary_array(array, thresh, value=0):
		"""
		Return a 2D binary array (mask) in which all pixels are either 0 or 1
			
		:param array: NumPy 2D array that we want to convert to binary values
		:param thresh: Values used for thresholding (inclusive)
		:param value: Output value when between the supplied threshold
		:return: Binary 2D array
		"""

		if value == 0:
			# Create an array of ones with the same shape and type as 
			# the input 2D array.
			binary = np.ones_like(array) 
				
		else:
			# Creates an array of zeros with the same shape and type as 
			# the input 2D array.
			binary = np.zeros_like(array)  
			value = 1
		
		# If value == 0, make all values in binary equal to 0 if the 
		# corresponding value in the input array is between the threshold 
		# (inclusive). Otherwise, the value remains as 1. Therefore, the pixels 
		# with the high Sobel derivative values (i.e. sharp pixel intensity 
		# discontinuities) will have 0 in the corresponding cell of binary.
		binary[(array >= thresh[0]) & (array <= thresh[1])] = value
		
		return binary
	
	@staticmethod
	def blur_gaussian(channel, ksize=3):
		"""
		Implementation for Gaussian blur to reduce noise and detail in the image
			
		:param image: 2D or 3D array to be blurred
		:param ksize: Size of the small matrix (i.e. kernel) used to blur
						i.e. number of rows and number of columns
		:return: Blurred 2D image
		"""

		return cv2.GaussianBlur(channel, (ksize, ksize), 0)

	@staticmethod
	def sobel(img_channel, orient='x', sobel_kernel=3):
		"""
		Find edges that are aligned vertically and horizontally on the image
			
		:param img_channel: Channel from an image
		:param orient: Across which axis of the image are we detecting edges?
		:sobel_kernel: No. of rows and columns of the kernel (i.e. 3x3 small matrix)
		:return: Image with Sobel edge detection applied
		"""
		# cv2.Sobel(input image, data type, prder of the derivative x, order of the
		# derivative y, small matrix used to calculate the derivative)
		if orient == 'x':
			# Will detect differences in pixel intensities going from 
				# left to right on the image (i.e. edges that are vertically aligned)
			sobel = cv2.Sobel(img_channel, cv2.CV_64F, 1, 0, sobel_kernel)
		if orient == 'y':
			# Will detect differences in pixel intensities going from 
			# top to bottom on the image (i.e. edges that are horizontally aligned)
			sobel = cv2.Sobel(img_channel, cv2.CV_64F, 0, 1, sobel_kernel)
		
		return sobel
	
	@staticmethod
	def threshold(channel, thresh=(128,255), thresh_type=cv2.THRESH_BINARY):
		"""
		Apply a threshold to the input channel
			
		:param channel: 2D array of the channel data of an image/video frame
		:param thresh: 2D tuple of min and max threshold values
		:param thresh_type: The technique of the threshold to apply
		:return: Two outputs are returned:
					ret: Threshold that was used
					thresholded_image: 2D thresholded data.
		"""
		# If pixel intensity is greater than thresh[0], make that value
		# white (255), else set it to black (0)
		return cv2.threshold(channel, thresh[0], thresh[1], thresh_type)
			
	def mag_thresh(self, image, sobel_kernel=3, thresh=(0, 255)):
		"""
		Implementation of Sobel edge detection
		
		:param image: 2D or 3D array to be blurred
		:param sobel_kernel: Size of the small matrix (i.e. kernel) 
							i.e. number of rows and columns
		:return: Binary (black and white) 2D mask image
		"""
		# Get the magnitude of the edges that are vertically aligned on the image
		sobelx = np.absolute(self.sobel(image, orient='x', sobel_kernel=sobel_kernel))
				
		# Get the magnitude of the edges that are horizontally aligned on the image
		sobely = np.absolute(self.sobel(image, orient='y', sobel_kernel=sobel_kernel))
		
		# Find areas of the image that have the strongest pixel intensity changes
		# in both the x and y directions. These have the strongest gradients and 
		# represent the strongest edges in the image (i.e. potential lane lines)
		# mag is a 2D array .. number of rows x number of columns = number of pixels
		# from top to bottom x number of pixels from left to right
		mag = np.sqrt(sobelx ** 2 + sobely ** 2)
		
		# Return a 2D array that contains 0s and 1s   
		return self.binary_array(mag, thresh)
	
	def isolate_lanes(self, image):
		""" Isolates the lane lines of the input image.
		
		:param image: The raw image input to the pipeline.
		:return: A binary image with the lane lines isolated.
		"""
		# White Color Mask
		lower = np.uint8([200, 200, 200])
		upper = np.uint8([255, 255, 255])
		white_mask = cv2.inRange(image, lower, upper)

		# Yellow Color Mask
		lower = np.uint8([80, 150,  0])
		upper = np.uint8([255, 255, 255])
		yellow_mask = cv2.inRange(image, lower, upper)

		# Combine Masks
		mask = cv2.bitwise_or(white_mask, yellow_mask)
		masked = cv2.bitwise_and(image, image, mask = mask)

		_, thresh_img = cv2.threshold(masked, 10, 255, cv2.THRESH_BINARY)

		return cv2.cvtColor(thresh_img, cv2.COLOR_BGR2GRAY)

	def region_selection(self, image):
		"""
		Determine and cut the region of interest in the input image.

		:param image: The input image from the pipeline.
		"""
		mask = np.zeros_like(image)   
		# Defining a 3 channel or 1 channel color to fill the mask with depending on the input image
		if len(image.shape) > 2:
			channel_count = image.shape[2]
			ignore_mask_color = (255,) * channel_count
		else:
			ignore_mask_color = 255

		bottom_left  = self.roi_points[1]
		top_left     = self.roi_points[0]
		bottom_right = self.roi_points[2]
		top_right    = self.roi_points[3]

		vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
		cv2.fillPoly(mask, vertices, ignore_mask_color)
		masked_image = cv2.bitwise_and(image, mask)
		return masked_image

	def calculate_car_position(self, print_to_terminal=False):
		"""
		Calculate the position of the car relative to the center
				 
		:param: print_to_terminal Display data to console if True       
		:return: Offset from the center of the lane
		"""
		# Assume the camera is centered in the image.
		# Get position of car in centimeters
		car_location = self.width / 2
 
		# Fine the x coordinate of the lane line bottom
		height = self.height
		bottom_left = self.left_fit[0]*height**2 + self.left_fit[
			1]*height + self.left_fit[2]
		bottom_right = self.right_fit[0]*height**2 + self.right_fit[
			1]*height + self.right_fit[2]
 
		center_lane = (bottom_right - bottom_left)/2 + bottom_left 
		center_offset = (np.abs(car_location) - np.abs(
			center_lane)) * self.XM_PER_PIX * 100


		# Display on terminal window and log
		if print_to_terminal == True:
			print(str(center_offset) + 'cm')

		self.center_offset = center_offset
		 
		return center_offset
 
	def calculate_curvature(self, print_to_terminal=False):
		"""
		Calculate the road curvature in meters.
 
		:param: print_to_terminal Display data to console if True
		:return: Radii of curvature
		"""
		# Set the y-value where we want to calculate the road curvature.
		# Select the maximum y-value, which is the bottom of the frame.
		y_eval = np.max(self.ploty)    
 
		# Fit polynomial curves to the real world environment
		left_fit_cr = np.polyfit(self.lefty * self.YM_PER_PIX, self.leftx * (
			self.XM_PER_PIX), 2)
		right_fit_cr = np.polyfit(self.righty * self.YM_PER_PIX, self.rightx * (
			self.XM_PER_PIX), 2)
						 
		# Calculate the radii of curvature
		left_curvem = ((1 + (2*left_fit_cr[0]*y_eval*self.YM_PER_PIX + left_fit_cr[
										1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
		right_curvem = ((1 + (2*right_fit_cr[
										0]*y_eval*self.YM_PER_PIX + right_fit_cr[
										1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
		 
		# Display on terminal window and log
		if print_to_terminal == True:
			print(left_curvem, 'm', right_curvem, 'm')

		self.left_curvem = left_curvem
		self.right_curvem = right_curvem
 
		return left_curvem, right_curvem        
				 
	def calculate_histogram(self, frame):
		"""
		Calculate the image histogram to find peaks in white pixel count
				 
		:param frame: The warped image
		:param plot: Create a plot if True
		"""
						 
		# Generate the histogram
		self.histogram = np.sum(frame[int(
							frame.shape[0]/2):,:], axis=0)
		return self.histogram
 
	def display_curvature_offset(self, frame):
		"""
		Display curvature and offset statistics on the image
				 
		:param: plot Display the plot if True
		:return: Image with lane lines and curvature
		"""
		image_copy = frame.copy()
 
		# cv2.putText(image_copy,'Curve Radius: '+str((
		# 	self.left_curvem+self.right_curvem)/2)[:7]+' m', (int((
		# 	5/600)*self.width), int((
		# 	20/338)*self.height)), cv2.FONT_HERSHEY_SIMPLEX, (float((
		# 	0.5/600)*self.width)),(
		# 	255,255,255),2,cv2.LINE_AA)
		# cv2.putText(image_copy,'Center Offset: '+str(
		# 	self.center_offset)[:7]+' cm', (int((
		# 	5/600)*self.width), int((
		# 	40/338)*self.height)), cv2.FONT_HERSHEY_SIMPLEX, (float((
		# 	0.5/600)*self.width)),(
		# 	255,255,255),2,cv2.LINE_AA)
						 
		return image_copy
		 
	def get_lane_line_previous_window(self, left_fit, right_fit, plot=False):
		"""
		Use the lane line from the previous sliding window to get the parameters
		for the polynomial line for filling in the lane line
		:param: left_fit Polynomial function of the left lane line
		:param: right_fit Polynomial function of the right lane line
		:param: plot To display an image or not
		"""
		# margin is a sliding window parameter
		margin = self.margin
 
		# Find the x and y coordinates of all the nonzero 
		# (i.e. white) pixels in the frame.         
		nonzero = self.warped_frame.nonzero()  
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
				 
		# Store left and right lane pixel indices
		left_lane_inds = ((nonzerox > (left_fit[0]*(
			nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (
			nonzerox < (left_fit[0]*(
			nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
		right_lane_inds = ((nonzerox > (right_fit[0]*(
			nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (
			nonzerox < (right_fit[0]*(
			nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))           
		self.left_lane_inds = left_lane_inds
		self.right_lane_inds = right_lane_inds
 
		# Get the left and right lane line pixel locations  
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds] 
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]  
 
		self.leftx = leftx
		self.rightx = rightx
		self.lefty = lefty
		self.righty = righty        
		 
		# Fit a second order polynomial curve to each lane line
		try:
			left_fit = np.polyfit(lefty, leftx, 2)
		except TypeError:
			left_fit = np.array([0,0,0])
			

		try:
			right_fit = np.polyfit(righty, rightx, 2) 
		except TypeError:
			right_fit = self.fallback_right
			
				 
		self.left_fit = left_fit
		self.right_fit = right_fit

		# Create the x and y values to plot on the image
		ploty = np.linspace(
			0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0]) 
		left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
		self.ploty = ploty
		self.left_fitx = left_fitx
		self.right_fitx = right_fitx
				 
		if plot==True:
				 
			# Generate images to draw on
			out_img = np.dstack((self.warped_frame, self.warped_frame, (
													 self.warped_frame)))*255
			window_img = np.zeros_like(out_img)
						 
			# Add color to the left and right line pixels
			out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
			out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [
																																		 0, 0, 255]
			# Create a polygon to show the search window area, and recast 
			# the x and y points into a usable format for cv2.fillPoly()
			margin = self.margin
			left_line_window1 = np.array([np.transpose(np.vstack([
																		left_fitx-margin, ploty]))])
			left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([
																		left_fitx+margin, ploty])))])
			left_line_pts = np.hstack((left_line_window1, left_line_window2))
			right_line_window1 = np.array([np.transpose(np.vstack([
																		 right_fitx-margin, ploty]))])
			right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([
																		 right_fitx+margin, ploty])))])
			right_line_pts = np.hstack((right_line_window1, right_line_window2))
						 
			# Draw the lane onto the warped blank image
			cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
			cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
			result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
						 
	def get_lane_line_indices_sliding_windows(self, frame):
		"""
		Get the indices of the lane line pixels using the 
		sliding windows technique.
				 
		:param: plot Show plot or not
		:return: Best fit lines for the left and right lines of the current lane 
		"""
		# Sliding window width is +/- margin
		margin = self.margin
 
		frame_sliding_window = frame.copy()
 
		# Set the height of the sliding windows
		window_height = int(self.warped_frame.shape[0]/self.no_of_windows)       
 
		# Find the x and y coordinates of all the nonzero 
		# (i.e. white) pixels in the frame. 
		nonzero = frame.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1]) 
				 
		# Store the pixel indices for the left and right lane lines
		left_lane_inds = []
		right_lane_inds = []
				 
		# Current positions for pixel indices for each window,
		# which we will continue to update
		leftx_base, rightx_base = self.histogram_peak()
		leftx_current = leftx_base
		rightx_current = rightx_base
 
		# Go through one window at a time
		no_of_windows = self.no_of_windows
				 
		for window in range(no_of_windows):
			 
			# Identify window boundaries in x and y (and right and left)
			win_y_low = self.warped_frame.shape[0] - (window + 1) * window_height
			win_y_high = self.warped_frame.shape[0] - window * window_height
			win_xleft_low = leftx_current - margin
			win_xleft_high = leftx_current + margin
			win_xright_low = rightx_current - margin
			win_xright_high = rightx_current + margin
			cv2.rectangle(frame_sliding_window,(win_xleft_low,win_y_low),(
				win_xleft_high,win_y_high), (255,255,255), 2)
			cv2.rectangle(frame_sliding_window,(win_xright_low,win_y_low),(
				win_xright_high,win_y_high), (255,255,255), 2)
 
			# Identify the nonzero pixels in x and y within the window
			good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
													(nonzerox >= win_xleft_low) & (
													 nonzerox < win_xleft_high)).nonzero()[0]
			good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
													 (nonzerox >= win_xright_low) & (
														nonzerox < win_xright_high)).nonzero()[0]
																												 
			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)
			right_lane_inds.append(good_right_inds)
				 
			# If you found > minpix pixels, recenter next window on mean position
			minpix = self.minpix
			if len(good_left_inds) > minpix:
				leftx_current = int(np.mean(nonzerox[good_left_inds]))
			if len(good_right_inds) > minpix:        
				rightx_current = int(np.mean(nonzerox[good_right_inds]))
										 
		# Concatenate the arrays of indices
		left_lane_inds = np.concatenate(left_lane_inds)
		right_lane_inds = np.concatenate(right_lane_inds)
 
		# Extract the pixel coordinates for the left and right lane lines
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds] 
		rightx = nonzerox[right_lane_inds] 
		righty = nonzeroy[right_lane_inds]
 
		# Fit a second order polynomial curve to the pixel coordinates for
		# the left and right lane lines
		
		try:
			left_fit = np.polyfit(lefty, leftx, 2)
		except TypeError:
			left_fit = np.array([0,0,0])
			

		try:
			right_fit = np.polyfit(righty, rightx, 2) 
		except TypeError:
			right_fit = self.fallback_right
			
		self.left_fit = left_fit
		self.right_fit = right_fit
 				 
		return self.left_fit, self.right_fit
 
	def get_line_markings(self, frame):
		"""
		Isolates lane lines.
	 
		:param frame: The camera frame that contains the lanes we want to detect
		:return: Binary (i.e. black and white) image containing the lane lines.
		"""
		# Convert the video frame from BGR (blue, green, red) 
		# color space to HLS (hue, saturation, lightness).
		hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
 
		################### Isolate possible lane line edges ######################
				 
		# Perform Sobel edge detection on the L (lightness) channel of 
		# the image to detect sharp discontinuities in the pixel intensities 
		# along the x and y axis of the video frame.             
		# sxbinary is a matrix full of 0s (black) and 255 (white) intensity values
		# Relatively light pixels get made white. Dark pixels get made black.
		_, sxbinary = self.threshold(hls[:, :, 1], thresh=(80, 255))
		sxbinary = self.blur_gaussian(sxbinary, ksize=3) # Reduce noise
				 
		# 1s will be in the cells with the highest Sobel derivative values
		# (i.e. strongest lane line edges)
		sxbinary = self.mag_thresh(sxbinary, sobel_kernel=3, thresh=(120, 255))
 
		######################## Isolate possible lane lines ######################

		# Perform binary thresholding on the S (saturation) channel 
		# of the video frame. A high saturation value means the hue color is pure.
		# We expect lane lines to be nice, pure colors (i.e. solid white, yellow)
		# and have high saturation channel values.
		# s_binary is matrix full of 0s (black) and 255 (white) intensity values
		# White in the regions with the purest hue colors (e.g. >80...play with
		# this value for best results).
		s_channel = hls[:, :, 2] # use only the saturation channel data
		_, s_binary = self.threshold(s_channel, (75, 255))
		 
		# Perform binary thresholding on the R (red) channel of the 
				# original BGR video frame. 
		# r_thresh is a matrix full of 0s (black) and 255 (white) intensity values
		# White in the regions with the richest red channel values (e.g. >120).
		# Remember, pure white is bgr(255, 255, 255).
		# Pure yellow is bgr(0, 255, 255). Both have high red channel values.
		_, r_thresh = self.threshold(frame[:, :, 2], thresh=(110, 255))
		
		# Lane lines should be pure in color and have high red channel values 
		# Bitwise AND operation to reduce noise and black-out any pixels that
		# don't appear to be nice, pure, solid colors (like white or yellow lane 
		# lines.)       
		rs_binary = cv2.bitwise_and(s_binary, r_thresh)
 
		### Combine the possible lane lines with the possible lane line edges ##### 
		# If you show rs_binary visually, you'll see that it is not that different 
		# from this return value. The edges of lane lines are thin lines of pixels.
		self.lane_line_markings = cv2.bitwise_or(rs_binary, sxbinary.astype(
															np.uint8))    
		return self.lane_line_markings
				 
	def histogram_peak(self):
		"""
		Get the left and right peak of the histogram
 
		Return the x coordinate of the left histogram peak and the right histogram
		peak.
		"""
		midpoint = int(self.histogram.shape[0]/2)
		leftx_base = np.argmax(self.histogram[:midpoint])
		rightx_base = np.argmax(self.histogram[midpoint:]) + midpoint
 
		# (x coordinate of left peak, x coordinate of right peak)
		return leftx_base, rightx_base
				  
	def perspective_transform(self, frame):
		"""
		Perform the perspective transform.
		:param: frame Current frame
		:param: plot Plot the warped image if True
		:return: Bird's eye view of the current lane
		"""
		# Calculate the transformation matrix
		self.transformation_matrix = cv2.getPerspectiveTransform(
			self.roi_points, self.desired_roi_points)
 
		# Calculate the inverse transformation matrix           
		self.inv_transformation_matrix = cv2.getPerspectiveTransform(
			self.desired_roi_points, self.roi_points)
 
		# Perform the transform using the transformation matrix
		self.warped_frame = cv2.warpPerspective(
			frame, self.transformation_matrix, (self.width, self.height), flags=(
		 cv2.INTER_LINEAR)) 
 
		# Convert image to binary
		(thresh, binary_warped) = cv2.threshold(
			self.warped_frame, 180, 255, cv2.THRESH_BINARY)           
		self.warped_frame = binary_warped
 	 
		return self.warped_frame        
		 
	def plot_roi(self, frame):
		"""
		Plot the region of interest on an image.

		:param: frame The current image frame
		:param: plot Plot the roi image if True
		"""
		overlay = frame.copy()
		# Overlay trapezoid on the frame
		this_image = cv2.polylines(overlay, np.int32([
			self.roi_points]), True, (147, 20, 255), 3)
 
		# Display the image
		cv2.imshow('ROI Image', this_image)
		 
	def get_host_lane(self, frame):
		return self.host_lane
		
	def detect_lanes(self, frame):
		""" Detect Lane Lines in an image. """
		try:
			isolated1 = self.get_line_markings(frame)
			isolated2 = self.isolate_lanes(frame)
			self.isolated_lane_lines = cv2.addWeighted(isolated1, 1, isolated2, 1, 0)
			self.cropped_lane_lines = self.region_selection(self.isolated_lane_lines)
			
			self.warped_frame = self.perspective_transform(self.cropped_lane_lines)
			self.histogram = self.calculate_histogram(self.warped_frame)
			self.left_fit, self.right_fit = self.get_lane_line_indices_sliding_windows(self.warped_frame)
			self.get_lane_line_previous_window(self.left_fit, self.right_fit)
		except Exception as e:
			raise e

	def overlay_detections(self, frame):
		"""
		Overlay lane lines on the original frame
		:param: Plot the lane lines if True
		:return: Lane with overlay
		"""
		overlay = frame.copy()

		# Generate an image to draw the lane lines on 
		warp_zero = np.zeros(self.warped_frame.shape).astype(np.uint8)
		color_warp = np.dstack((warp_zero, warp_zero, warp_zero))       
				 
		# Recast the x and y points into usable format for cv2.fillPoly()
		pts_left = np.array([np.transpose(np.vstack([
												 self.left_fitx, self.ploty]))])
		pts_right = np.array([np.flipud(np.transpose(np.vstack([
													self.right_fitx, self.ploty])))])
		pts = np.hstack((pts_left, pts_right))

		# Find the values that define the line of best fit
		# TODO: This can be improved with something like RANSAC
		
		# Take the points that define the line.
		left_p1_x = int(self.left_fitx[0])
		left_p1_y = 0
		left_p2_x = int(self.left_fitx[-1])
		left_p2_y = 480

		right_p1_x = int(self.right_fitx[0])
		right_p1_y = 0
		right_p2_x = int(self.right_fitx[-1])
		right_p2_y = 480

		# Determine what points are the top and bottom (for midpoint calculation)
		if (left_p1_y > left_p2_y):
			left_top_x = left_p2_x
			left_top_y = left_p2_y
			left_bottom_x = left_p1_x
			left_bottom_y = left_p1_y

		else:
			left_top_x = left_p1_x
			left_top_y = left_p1_y
			left_bottom_x = left_p2_x
			left_bottom_y = left_p2_y

		if (right_p1_y > right_p2_y):
			right_top_x = right_p2_x
			right_top_y = right_p2_y
			right_bottom_x = right_p1_x
			right_bottom_y = right_p1_y

		else:
			right_top_x = right_p1_x
			right_top_y = right_p1_y
			right_bottom_x = right_p2_x
			right_bottom_y = right_p2_y

		midpoint_top_x = int((right_top_x + left_top_x) / 2)
		midpoint_top_y = left_top_y if left_top_y < right_top_y else right_top_y
		midpoint_bottom_x = int(color_warp.shape[1] / 2)
		midpoint_bottom_y = int(color_warp.shape[0])

		# Draw lane on the warped blank image
		cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
		cv2.polylines(color_warp, np.int_([pts]), True, (0, 0, 255), 10)

		cv2.line(color_warp, (left_bottom_x, left_bottom_y), (left_top_x, left_top_y), (255, 0, 0), 10)
		cv2.line(color_warp, (right_bottom_x, right_bottom_y), (right_top_x, right_top_y), (255, 0, 0), 10)
		cv2.line(color_warp, (midpoint_bottom_x, midpoint_bottom_y), (midpoint_top_x, midpoint_top_y), (0, 0, 255), 10)

		# Warp the blank back to original image space using inverse perspective matrix
		newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix, (self.width, self.height))

		# TODO: Remap the line points to the camera coordinate 

		# Combine the result with the original image
		result = cv2.addWeighted(overlay, 1, newwarp, 0.6, 0)
		self.lanes_top_view = color_warp
		self.lane_pts_top_view = pts
		self.lanes_camera_view = result
		self.target_x = midpoint_top_x
		return result           
		
	def print_detections(self):
		for line in self.lane_lines:
			print(f'Lane: {line.type}\tColor: {line.color}\tCurvature: {line.curvature}')
