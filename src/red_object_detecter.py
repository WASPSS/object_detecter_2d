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
from geometry_msgs.msg import PointStamped
from object_detecter_2d.msg import object_loc
from pyimagesearch.transform import four_point_transform
import tf
from std_msgs.msg import String
from math import hypot

def Longest_Length(approxcontour):
	approxcontour = np.concatenate((approxcontour,[approxcontour[0]]))
	#print (approxcontour)
	#formula to find difference between two points 
	ptdiff = lambda (p1,p2): (p1[0]-p2[0], p1[1]-p2[1])
	diffs = map(ptdiff, zip(approxcontour,approxcontour[1:]))
	dist_ = []
	for d in diffs:
		dist_.append(hypot(*d))

	#print ('dist ', dist_)
	LongestSide = max(dist_)
	return LongestSide



class object_detecter:
	def __init__(self):
		self.object_location_pub = rospy.Publisher("/object_location", object_loc, queue_size =1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage, self.callback)
		self.tl = tf.TransformListener()
		self.pub = rospy.Publisher('/chatter', String, queue_size=1, latch=True)
		#time.sleep(0.1)
		#self.tfp = tf.transformPoint()
	def callback(self,data):
		focal_leng = 570.34222
		np_arr = np.fromstring(data.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		img_cpy = cv_image.copy()
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		#Red_Thresholds
		lower_red1 = np.array([0, 100, 100])
		upper_red1 = np.array([10, 255,255])	
		lower_red2 = np.array([160,100,100])
		upper_red2 = np.array([179,255,255])
		
		#Blue Thresholds
		lower_blue = np.array([104,120,120])	
		upper_blue = np.array([143,255,255])	
		lower_green = np.array([60,60,46])	
		upper_green = np.array([97,255,255])	

		# Threshold the HSV image to get only blue colors
		# mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
		# For the color used in testing only Upper threshold was sufficient, 
		# if the final image needs a different threshold  add lower mask also
		#mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
		#mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
		mask2 = cv2.inRange(hsv, lower_green, upper_green)
		#imgthreshed = cv2.add(mask1,mask2)

		contours, hierarchy = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		for x in range (len(contours)):
			contourarea = cv2.contourArea(contours[x]) #get area of contour
			if contourarea > 600: #Discard contours with a small area as this may just be noise
				arclength = cv2.arcLength(contours[x], True)
				approxcontour = cv2.approxPolyDP(contours[x], 0.02 * arclength, True)
				rect = cv2.minAreaRect(contours[x])
				obj_x = int(rect[0][0])
				obj_y = int(rect[0][1])
				
				#Check for Square
				if len(approxcontour) == 4:
					#print ('Length ', len(approxcontour))
					cv2.drawContours(cv_image,[approxcontour],0,(0,255,255),2)
					#print ('x : ',  boxcentrex, 'y : ', boxcentrey)
					approxcontour = approxcontour.reshape((4,2))
					LongestSide = Longest_Length(approxcontour)
					Distance = (focal_leng*11.5)/LongestSide #focal length x Actual Border width / size of Border in pixels
					print ("Distance= " , Distance)

				#Check for Triangle
				elif len(approxcontour) == 3:
					#print ('Length ', len(approxcontour))
					cv2.drawContours(cv_image,[approxcontour],0,(0,255,255),2)
					approxcontour = approxcontour.reshape((3,2))
					LongestSide = Longest_Length(approxcontour)
					Distance = (focal_leng*12.4)/LongestSide #focal length x Actual Border width / size of Border in pixels
					print ("Distance= " , Distance)

				#Check for Star
				elif len(approxcontour) == 8:
					print ('Length ', len(approxcontour))
					cv2.drawContours(cv_image,[approxcontour],0,(0,255,255),2)
					approxcontour = approxcontour.reshape((8,2))
					LongestSide = Longest_Length(approxcontour)
					Distance = (focal_leng*5.5)/LongestSide #focal length x Actual Border width / size of Border in pixels
					print ("Distance= " , Distance)	
				#Move to next Contour
				else :
					continue

		
			'''
				#Calculate Cordinates wrt to Camera, convert to Map
				#Coordinates and publish message for storing 	
				obj_cam_x = ((obj_x - 319.5)*Distance)/focal_leng
				obj_cam_y = ((obj_y - 239.5)*Distance)/focal_leng

				P = PointStamped()
				P.header.stamp = data.header.stamp
				P.header.frame_id = 'camera_rgb_optical_frame'
				P.point.x = obj_cam_x/100.0
				P.point.y = obj_cam_y/100.0
				P.point.z = Distance /100.0
				
				#Transform Point 
				trans_pt = self.tl.transformPoint('/map', P)

				#print ('trans', trans_pt)

				#Create a publisher 
				obj_info_pub = object_loc()
				obj_info_pub.ID = 27 #ID for Circle need to be chnaged
				obj_info_pub.point.x = trans_pt.point.x
				obj_info_pub.point.y = trans_pt.point.y
				obj_info_pub.point.z = trans_pt.point.z

				self.object_location_pub.publish(obj_info_pub) 
			#'''

		cv2.imshow("Image",np.hstack([img_cpy, cv_image]))
		#cv2.imshow("Final Frame", hsv[:,:,0])
		#cv2.imshow("Gray", gray)
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