#!/usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import CompressedImage,Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
import random as rng

rng.seed(12345)

img_preview_time = 200
global contoured_img
contoured_img = np.zeros((768, 1024, 3))



def getRedMask(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # join my masks
    mask = mask0+mask1

    # set my output img to zero everywhere except my mask
    output_img = img.copy()
    #output_img[np.where(mask==0)] = 0
    output_img = cv2.bitwise_and(output_img, output_img, mask=mask)
    return output_img

#https://stackoverflow.com/questions/47483951/how-to-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-image
"""
From (64,10/25,0/25) to [110,255,225/200]
(64,45,25) to [110,255,235]
"""
def getGreenMask(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower mask (0-10)     (36,25,25) to [180,255,255] for whole circle
    lower_green = np.array((64,64,25))# Cable green:[36,25,25]  100
    upper_green = np.array([110,255,235])# Cable green: 180,[70,255,255], 245
    #[76,255,76]
    mask = cv2.inRange(img_hsv, lower_green, upper_green)

    # set my output img to zero everywhere except my mask
    output_img = img.copy()
    output_img[np.where(mask==0)] = 0
    #output_img = cv2.bitwise_and(output_img, output_img, mask=mask)
    return output_img

"""
(36,75 or 100,30)) - [180,255,235]

"""
def getMainCircle(img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower mask (0-10)     (36,25,25) to [180,255,255] for whole circle
    # lower_green = np.array((64,64,25))# Cable green:[36,25,25]  100
    # upper_green = np.array([110,255,235])# Cable green: 180,[70,255,255], 245
    lower_green = np.array((36,100,30))   #(64,64,25)
    upper_green = np.array([180,255,255])
    #[76,255,76]
    mask = cv2.inRange(img_hsv, lower_green, upper_green)

    # set my output img to zero everywhere except my mask
    output_img = img.copy()
    output_img[np.where(mask==0)] = 0
    #output_img = cv2.bitwise_and(output_img, output_img, mask=mask)
    return output_img

#https://stackoverflow.com/questions/52107379/intensify-or-increase-saturation-of-an-image
def onlyGreens(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    greenMask = cv2.inRange(hsv, (26, 10, 30), (97, 100, 255))

    hsv[:,:,1] = greenMask 


    back = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return back

def increaseSaturation(img, amount=50):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_hsv[:,:,1] += amount
    return cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)

def contrast(img):
    res = img.shape
    print("Res: ",res)
    tileGridSize = (res[0]/64,res[1]/64)#(res[0]/64,res[1]/64)#, (8,8)
    clahe = cv2.createCLAHE(clipLimit=8.0, tileGridSize=tileGridSize)
    img_YCrCb = cv2.cvtColor(img,cv2.COLOR_BGR2YCrCb)
    contrastedImg = clahe.apply(img_YCrCb[:,:,0])
    contrastedImg = np.dstack([contrastedImg,img_YCrCb[:,:,1],img_YCrCb[:,:,2]])
    print("contrastedImg shape: ", contrastedImg.shape)
    contrastedImg = cv2.cvtColor(contrastedImg,cv2.COLOR_YCrCb2BGR)
    return contrastedImg


def getCdf(img):
    hist,bins = np.histogram(img.flatten(),256,[0,256])
    cdf = hist.cumsum()
    cdf_normalized = cdf * float(hist.max()) / cdf.max()
    return cdf,cdf_normalized
def showHistogram(img):
    _,cdf_normalized = getCdf(img)
    plt.plot(cdf_normalized, color = 'b')
    plt.hist(img.flatten(),256,[0,256], color = 'r')
    plt.xlim([0,256])
    plt.legend(('cdf','histogram'), loc = 'upper left')
    plt.show()
    
def numpyCdfContrast(img):
    cdf,_ = getCdf(img)
    cdf_m = np.ma.masked_equal(cdf,0)
    cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())
    cdf = np.ma.filled(cdf_m,0).astype('uint8')
    return cdf[img]
    
# The size of the erosion and dluation depends on how big it is in the camera. Maybe can detect the average size of islands and decide on kernel size (or even to run this function)    
def removingIslands(img):
    morphed_img = img
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8)) #(6,6) (24,24), (32,32)-> WOrked pretty well, (128,128) 
    morphed_img = cv2.morphologyEx(morphed_img, cv2.MORPH_OPEN,morph_kernel, iterations = 1) #Remove outside specks
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))#(3,3), (7,7), (15,15), (10,10), (64,64), (168,168)   #MORPH_RECT, MORPH_ELLIPSE, MORPH_CROSS
    morphed_img = cv2.morphologyEx(img, cv2.MORPH_CLOSE,morph_kernel, iterations = 1)   #remove internal specks
    morphed_mask = cv2.cvtColor(morphed_img, cv2.COLOR_BGR2GRAY)
    return cv2.bitwise_and(img, img, mask=morphed_mask)

def closingCircleTarget(img):
    #try some logic on the biggest island vs image res, then form an appropriate sized kernel

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (24,24))#(32,32), (64,64), (128,128), (168,168))  #MORPH_RECT, MORPH_ELLIPSE, MORPH_CROSS
    morphed_img = cv2.morphologyEx(img, cv2.MORPH_CLOSE,morph_kernel, iterations = 1)   #remove internal specks
    morphed_mask = cv2.cvtColor(morphed_img, cv2.COLOR_BGR2GRAY)
    return morphed_mask# cv2.bitwise_and(img, img, mask=morphed_mask)


def cannyOutput(img):
    thresh = 50
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    canny_img = cv2.Canny(img_gray,50,200)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(16,16)) #(16,16)
    dilated = cv2.dilate(canny_img, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12)) #(16,16)
    eroded = cv2.erode(dilated, kernel)
    return eroded #25,150

def cannyMask(src_img):
    img = src_img.copy()
    canny_img = cannyOutput(img)
    mask = np.zeros(src_img.shape,dtype='uint8')
    print(mask.shape)
    _,thresh = cv2.threshold(canny_img,127,255, cv2.THRESH_BINARY)
    _,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)   #RETR_LIST
    img = cv2.drawContours(img, contours, -1, (255 , 255 , 255),thickness=cv2.FILLED)
    # mask = cv2.bitwise_not(mask)
    # img2gray = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
    # ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
    # result= cv2.bitwise_and(src_img,src_img,mask=mask)
    for c in contours:
        c = cv2.approxPolyDP(c,32,True)
        color = (rng.randint(0,256))
        rect = cv2.boundingRect(c)
        x,y,w,h = rect
        cv2.rectangle(img, (x,y), (x + w, y + h), color,3)
        cv2.putText(img, str(cv2.contourArea(c)), (x + 10, y-10), 0, 0.7, (0,255,0))
    return img


def getContours(src_img,mask,color):
    #img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    src_img = src_img.copy()
    _,thresh = cv2.threshold(mask,127,255, cv2.THRESH_BINARY)
    _,contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10] #cull areas less than certain number

    img_contours =  cv2.drawContours(src_img, sorted_contours, -1,color,3)
    for c in sorted_contours:
        c = cv2.approxPolyDP(c,16,True) #3
        color = (rng.randint(0,256))
        rect = cv2.boundingRect(c)
        x,y,w,h = rect
        cv2.rectangle(img_contours, (x,y), (x + w, y + h), color,3)
        cv2.putText(img_contours, str(cv2.contourArea(c)), (x + 10, y-10), 0, 0.7, (0,255,0))
    return img_contours
def img_callback(msg):
    print('received image of type: "%s"' % msg.format)
    global contoured_img
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    contrasted_img = contrast(image_np)
    #greener_img = onlyGreens(image_np)
    only_red_img = getRedMask(image_np)
    only_green_img = getGreenMask(contrasted_img)
    circle_only_img = getMainCircle(contrasted_img)
    #redAndGreen_Img = only_red_img + only_green_img
    morphed_img = removingIslands(circle_only_img)
    #canny_img = cannyOutput(image_np)
    #redContour = getContours(contrasted_img,cannyOutput(removingIslands(only_red_img)),(0,255,0)) 
    #contoured_img = getContours(contrasted_img,cannyOutput(removingIslands(only_red_img)),(0,255,0))  #canny_img, morphed_img
    #greenContour = getContours(contrasted_img,cannyOutput(removingIslands(only_green_img)),(0,0,255))
    contoured_img = getContours(contrasted_img,cannyOutput(morphed_img),(255,0,0))
    canny_contour_img = cannyMask(image_np)
    #contoured_img = getContours(contoured_img,canny_img,(255,255,0))
    # circle_only_mask = closingCircleTarget(morphed_img)
    # circle_only_img = cv2.bitwise_and(contrasted_img, contrasted_img, mask=circle_only_mask)
    
    #more_saturated_img = increaseSaturation(image_np)

    #cv2.imshow('orig_img', image_np)
    #cv2.waitKey(img_preview_time)
    
    # cv2.imshow('cv_img', redAndGreen_Img)
    # cv2.waitKey(img_preview_time)
    #cv2.imshow("Saturated Img",more_saturated_img)
    cv2.imshow("Closed_img",morphed_img)
    cv2.imshow("Red only", removingIslands(only_red_img))
    cv2.imshow("Green only", removingIslands(only_green_img))
    cv2.imshow("Canny img",canny_contour_img)
    cv2.imshow("With contours",contoured_img)
    #cv2.imshow("CIRCLE ONLY", circle_only_img)
    #cv2.imshow("Greener Img",greener_img)
    #cv2.imshow("Contrasted Img", contrasted_img)
    cv2.waitKey(img_preview_time)

"""
contoured_img is numpy arr
"""
def image_msg_create():
    global contoured_img
    im = contoured_img.copy()
    #im = np.array(im)
    height, width,_ = im.shape
    
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = height
    msg.width = width
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = 3 * width
    msg.data = np.array(im).tobytes()
    return msg

def compressed_image_msg_create():
    global contoured_img
    print(contoured_img.shape)
    im = contoured_img.copy()
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    compressed_buffer = cv2.imencode('.jpg',im)
    msg.data = compressed_buffer
    return msg


def runPublishers():
    pub = rospy.Publisher('/detected/debug_img', Image, queue_size=10)
    pub2 = rospy.Publisher('/detected/debug_img_compressed', CompressedImage, queue_size=10)
    pub.publish(image_msg_create())
    pub2.publish(compressed_image_msg_create())
    while not rospy.is_shutdown():
        #print("MIAO") #get the numpy array and convert to whatever format
        pub.publish(image_msg_create())
        pub2.publish(compressed_image_msg_create())
    print("Finished publishing")
if __name__ == '__main__':
    rospy.init_node('img_processor', anonymous=True)
    rospy.Subscriber("/auv/bot_cam/image_color/compressed",CompressedImage,img_callback)
    try:
        runPublishers()
        #rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()