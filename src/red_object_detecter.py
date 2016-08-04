#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import time
from geometry_msgs.msg import Point
from object_detecter_2d.msg import object_loc
from pyimagesearch.transform import four_point_transform

def ReformContours(contours):
		contours = contours.reshape((4,2))
		contoursnew = np.zeros((4,2),dtype = np.float32)
		add = contours.sum(1)
		contoursnew[0] = contours[np.argmin(add)]
		contoursnew[2] = contours[np.argmax(add)]
		diff = np.diff(contours,axis = 1)
		contoursnew[1] = contours[np.argmin(diff)]
		contoursnew[3] = contours[np.argmax(diff)]
		return contoursnew


class object_detecter:
	def __init__(self):
		self.object_location_pub = rospy.Publisher("/object_location", object_loc, queue_size =1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage, self.callback)
	
	def callback(self,data):
		'''
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		'''
		np_arr = np.fromstring(data.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		cv2.imshow("sdsd", cv_image)
		
		#(rows,cols,channels) = cv_image.shape
		#print ('rows ', rows, ' coloumns ', cols)
		#hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		lower_red1 = np.array([0, 100, 100])
		upper_red1 = np.array([10, 255, 255])	
		lower_red2 = np.array([160, 100, 100])
		upper_red2 =np.array([179, 255, 255])	

		    # Threshold the HSV image to get only blue colors
		mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
		mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
		imgthreshed = cv2.add(mask1,mask2)

		contours, hierarchy = cv2.findContours(imgthreshed,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for x in range (len(contours)):
			contourarea = cv2.contourArea(contours[x]) #get area of contour
			if contourarea > 400: #Discard contours with a small area as this may just be noise
				arclength = cv2.arcLength(contours[x], True)
				approxcontour = cv2.approxPolyDP(contours[x], 0.08 * arclength, True)
				if len(approxcontour) == 4:
					cv2.drawContours(cv_image,[approxcontour],0,(0,255,255),2)
					
					rect = cv2.minAreaRect(contours[x])
					
					boxcentrex = int(rect[0][0])
					boxcentrey = int(rect[0][1])

					print ('x : ',  boxcentrex, 'y : ', boxcentrey)
					
					reformedcontour = ReformContours(approxcontour)
					#warp = four_point_transform(cv_image.copy(), approxcontour.reshape(4, 2))
					
					#cv2.imshow("extract", warp)
					#cv2.waitKey(1)
					#extract_obj = warp.copy()

					leftedge = reformedcontour[3][1] - reformedcontour[0][1]
					rightedge = reformedcontour[2][1] - reformedcontour[1][1]
					topedge = reformedcontour[1][0] - reformedcontour[0][0]
					bottomedge = reformedcontour[2][0] - reformedcontour[3][0]
					print ('Side Leghts',leftedge,rightedge,topedge,bottomedge )


					if leftedge > rightedge:
						LongestSide = leftedge
					else:
						LongestSide = rightedge
					if topedge > LongestSide:
						LongestSide = topedge
					if bottomedge > LongestSide:
						LongestSide = bottomedge
					Distance = (640.00*8)/LongestSide #focal length x Actual Border width / size of Border in pixels
					print ("Distance= " + str(Distance))


		cv2.imshow("RGB", cv_image)
		#cv2.imshow("HSV", hsv)
		
		cv2.waitKey(1)


def main(args):
	rospy.init_node('Red_Object_Detecter', anonymous = False)
	ic = object_detecter()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting Down Red_Object_Detecter')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)