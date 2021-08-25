#!/usr/bin/env python3

# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

import numpy as np

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('lane_detection', anonymous=True)
bridge = CvBridge()
masked_publisher = rospy.Publisher("/lane_detection/mask", Image, queue_size=10)

def RGB_to_BGR(img):
	# converting RGB to BGR
	image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
	return image

def RGB_to_Grayscale(img):
	grayscale_img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	return grayscale_img


def region_of_interest(img):
    # Define a blank matrix that matches the image height/width.
    mask = np.zeros_like(img)
    # Retrieve the number of color channels of the image.

    match_mask_color = 255
    (height, width) = img.shape
    # Fill polygon
    points = np.array([[[0,height],
        [int(width/4),int(height/2.6)],
        [int(width/2),int(height/5.5)],
        [int(width - (width/4)),int(height/2.6)],
        [width,height]]]) 
    cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)

    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def Lane_detect(frame):
    hsv=cv2.cvtColor(frame,cv2. COLOR_BGR2HSV)

    low_grey=np.array([0,0,0])
    up_grey=np.array([150,150,100])
    mask=cv2.inRange(hsv,low_grey,up_grey)
    edges=cv2.Canny(mask,75,150)

    gray_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(mask, (5, 5), 20)
    canny = cv2.Canny(blur, 50, 150)
    
    cropped = region_of_interest(blur)
    image_message = bridge.cv2_to_imgmsg(cropped, encoding="passthrough")
    masked_publisher.publish(image_message)


def image_callback(img_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    cv_image=RGB_to_BGR(cv_image)
    Lane_detect(cv_image)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)


# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()

print("End of the Program")