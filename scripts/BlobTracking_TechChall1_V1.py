#!/usr/bin/env python
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String
#from beaverworks77.msg import blob as BlobMsg

from cv_bridge import CvBridge, CvBridgeError
import threading

import os

import time

class ColorTracker:
    def __init__(self, debugging):
        self.node_name = "ColorTracker"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
	RGB = None
	self.shape = 'none'

        #self.pub_detection = rospy.Publisher("/detection", BlobMsg, queue_size=10)

        self.debugging = debugging
	
        self.bridge = CvBridge()
#Counting mech for images
	self.redImageCount = 0
	self.greenImageCount = 0
	self.yellowImageCount = 0
	self.blueImageCount = 0
	self.pinkImageCount = 0
	self.hsv = 0

        rospy.loginfo("[%s] Initialized." %(self.node_name))
		
	#os.mkdir(~/racecar-ws/challenge_photos)
#say hey we have an image
	self.pub_type = rospy.Publisher("/exploration_challenge", String, queue_size=10)

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def sortCont(self,contours):
	largestCont = None
	largestContAr = 500
	for i in contours:
	    thisArea = cv2.contourArea(i)
	    if thisArea > largestContAr:
		largestCont = i
		largestContAr = thisArea
	return largestCont

    def detection(self, img):
#Make image useful
	RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#HSV values for each color
        green_lower = np.array([50, 100, 75])
        green_upper = np.array([70, 255, 255])
        red_lower = np.array([175, 50, 75])
        red_upper = np.array([180, 200, 255])
        yellow_lower = np.array([25, 50, 150])
        yellow_upper = np.array([45, 150, 255])
        blue_lower = np.array([110, 100, 75])
        blue_upper = np.array([130, 200, 255])
        pink_lower = np.array([170, 30, 75])
        pink_upper = np.array([180, 200, 255])

#Create mask of each color
        green_mask = cv2.inRange(img, green_lower, green_upper)
        red_mask = cv2.inRange(img, red_lower, red_upper)
        yellow_mask = cv2.inRange(img, yellow_lower, yellow_upper)
        blue_mask = cv2.inRange(img, blue_lower, blue_upper)
        pink_mask = cv2.inRange(img, pink_lower, pink_upper)
#Combine all masks
        #maskOne = cv2.add(green_mask, red_mask, yellow_mask, blue_mask, pink_mask)
	mask_all = cv2.add(green_mask, cv2.add(red_mask, cv2.add(yellow_mask, blue_mask)))
        mask_all = cv2.erode(mask_all, (3,3), iterations=1)

	#for DEBUG, switch mask_x here
	contours = cv2.findContours(mask_all, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE)[0]
	largest = self.sortCont(contours)
	blob_color = None
	if largest != None:
#Find largest contour, find its center, find the HSV value
		centre = cv2.moments(largest)
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		blob_color = hsv[int(centre['m01'] / centre['m00']), int(centre['m10'] / centre['m00']), 0]
		cv2.drawContours(img, largest, -1, (0, 0, 255), 2)
		print blob_color

#Check if countour HSV value is in range for color, save image
	if blob_color != None:
	    if blob_color > 50 and blob_color < 70: #green found
	        os.chdir('/home/racecar/challenge_photos/') #Where we want the images saved to
	        greenName = "green" + self.shape + str(self.greenImageCount) + ".png"
	        cv2.imwrite(greenName, img) #Saves image to car
	        self.greenImageCount += 1 #Add naming scheme, prevents overwright of image
	        self.pub_type.publish(greenName) #Tells what image was found

	    if blob_color > 175 or blob_color < 15: #red found
		os.chdir('/home/racecar/challenge_photos/')
		redName = "red" + self.shape + str(self.redImageCount) + ".png"
		cv2.imwrite(redName, img)
		self.redImageCount += 1
		self.pub_type.publish(redName)
	    
	    if blob_color > 25 and blob_color < 35: #yellow found
	    	os.chdir('/home/racecar/challenge_photos/')
	    	yellowName = "yellow" + self.shape + str(self.yellowImageCount) + ".png"
	    	cv2.imwrite(yellowName, img)
	    	self.yellowImageCount += 1
	   	self.pub_type.publish(yellowName)
       
	    if blob_color > 110 and blob_color < 130: #blue found
		os.chdir('/home/racecar/challenge_photos/')
		blueName = "Blue" + self.shape + str(self.blueImageCount) + ".png"
		cv2.imwrite(blueName, img)
		self.blueImageCount += 1
		self.pub_type.publish(blueName)

#	    if blob_color > 146 and blob_color < 165: #pink found
#		os.chdir('/home/racecar/challenge_photos/')
#		Kitty = "Image" + str(self.pinkImageCount)
#		cv2.imwrite(Kitty, img)
#		self.pinkImageCount += 1
#		self.pub_type.publish("Image" + self.shape + " " + Kitty)

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        self.detection(image_cv)
        
        if self.debugging:
            try:
                self.pub_image.publish(\
                        self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
            except:
		pass
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('ColorTracker')
    e = ColorTracker(True)
    rospy.spin()
