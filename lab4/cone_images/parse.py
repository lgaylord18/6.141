import numpy as np
import math
import os
import cv2
import time

coneLower = (5,100,100)
coneUpper = (20,256,256)		


def mask_f(image):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, coneLower, coneUpper)
	mask = cv2.erode(mask, None, iterations=0)
	mask = cv2.dilate(mask, None, iterations=4)
	return mask

def findCM2(image,mask):
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	if len(cnts) > 0:
		    c = max(cnts, key=cv2.contourArea)
		    x,y,w,h = cv2.boundingRect(c)
		    M = cv2.moments(c)
		    # ratio = (self.height-(center[1]+include_range[0]))/self.height
            # distance = math.tan(ratio*math.pi/3.0+math.pi/3.0)*self.camHeight
		    center = (int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"]))
		    cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),4)
		    cv2.circle(image,center,5,(0,0,255),-1)
	return center



def findCM(mask):
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	if len(cnts) > 0:
		    c = max(cnts, key=cv2.contourArea)
		    # x,y,w,h = cv2.boundingRect(c)
		    M = cv2.moments(c)
		    # ratio = (self.height-(center[1]+include_range[0]))/self.height
            # distance = math.tan(ratio*math.pi/3.0+math.pi/3.0)*self.camHeight
		    center = (int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"]))
		    # cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),4)
		    # cv2.circle(image,center,5,(0,0,255),-1)
	return center

def processImage(image,include_range):
	image_cut = image[int(include_range[0]*len(image)):int(include_range[1]*len(image))] #get the lower pixels
	mask = mask_f(image_cut) #get the mask for the selected pixels
	center = findCM(mask) #find the center of mass of the larges contour
	if center:
		left = -320+center[0]
		ratio = (360-(center[1]+include_range[0]*len(image)))/360
		distance = math.tan(ratio*math.pi/2.0+math.pi/4.0)*.16
		theta = math.atan(left*math.tan(math.pi/4)/(320)) #angle to center of mass
	else:
		theta = 0
		distance = 0.5

		#rospy.loginfo(theta)
	return (distance,theta)


def breakDown()
	for fileName in os.listdir(os.getcwd()):
		if fileName[-4:] == '.png' and fileName[0] == "p":
			image = cv2.imread(fileName)
			mask = mask_f(image)
			print len(image),len(image[0])
			center = findCM2(image, mask)
			if center:
				left = -356+center[0]
				ratio = (376-(center[1]))/376.0
				print ratio,center
				distance = math.tan(ratio*math.pi/2.0+math.pi/4.0)*.16
				theta = math.atan(left*math.tan(math.pi/4)/(320)) #angle to center of mass
			else:
				theta = 0
				distance = 0.5

			print fileName[-7],distance
			cv2.imshow("Frame",image)
			cv2.waitKey(1)
			time.sleep(1)



def breakDown2()
	for fileName in os.listdir(os.getcwd()):
		if fileName[-4:] == '.png' and fileName[0] == "l":
			image = cv2.imread(fileName)
			im1 = image[:len(image)//2]
			im2 = image[len(image)//2:5*len(image)//8]
			im3 = image[5*len(image)//8:3*len(image)//4]
			im4 = image[3*len(image)//4:]

			mask2 = mask_f(im2)
			mask4 = mask_f(im4)

			findCM2(im2,mask2)
			findCM2(im4,mask4)

			a = np.zeros(image.shape,dtype='uint8')
			a[:len(image)//2] = im1
			a[len(image)//2:5*len(image)//8] = im2
			a[5*len(image)//8:3*len(image)//4] = im3
			a[3*len(image)//4:] = im4
			cv2.imshow("Frame",a)
			cv2.waitKey(1)
			time.sleep(15)



