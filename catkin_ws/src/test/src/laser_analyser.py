#!/usr/bin/env python

# laser_analyser.py created by Capt Tim Chisholm on 11 Jan 17 for RMC EE503

from __future__ import print_function
import roslib
roslib.load_manifest('lasertrackpkg')
import sys
import rospy
import cv2
import numpy
import argparse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lasertrackpkg.msg import laser_location_msg

class image_converter:
 
    #def __init__ adapted from "python-laser-tracker" from Brad Montgomery, Aug 2016
    #https://github.com/bradmontgomery/python-laser-tracker
    def __init__(self):
        self.cam_width = 640
        self.cam_height = 480
        self.hue_min = 20
        self.hue_max = 160
        self.sat_min = 100
        self.sat_max = 255
        self.val_min = 200
        self.val_max = 256    
        self.image_pub = rospy.Publisher("/lasertrackpkg/image_with_laser_track",Image)
        self.target_pub = rospy.Publisher("/lasertrackpkg/laser_location", laser_location_msg)    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.bridge = CvBridge()
        self.channels = {'hue': None,'saturation': None,'value': None,'laser': None}    
  
    #def threshold_image adapted from "python-laser-tracker" from Brad Montgomery, Aug 2016
    #https://github.com/bradmontgomery/python-laser-tracker
    def threshold_image(self, channel):
        if channel == "hue":
            minimum = self.hue_min
            maximum = self.hue_max
        elif channel == "saturation":
            minimum = self.sat_min
            maximum = self.sat_max
        elif channel == "value":
            minimum = self.val_min
            maximum = self.val_max
        (t, tmp) = cv2.threshold(self.channels[channel], maximum, 0, cv2.THRESH_TOZERO_INV)
        (t, self.channels[channel]) = cv2.threshold(tmp, minimum, 255, cv2.THRESH_BINARY)
        if channel == 'hue':
            # only works for filtering red color because the range for the hue is split
            self.channels['hue'] = cv2.bitwise_not(self.channels['hue'])  
  
    #def track adapted from "python-laser-tracker" from Brad Montgomery, Aug 2016
    #https://github.com/bradmontgomery/python-laser-tracker
    def track(self, frame, mask):
        """
        Track the position of the laser pointer.

        Code taken from
        http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        """
        center = None
        countours = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

        # only proceed if at least one contour was found
        if len(countours) > 0:
            # find the largest contour in the mask, then use it to compute the 
            # minimum enclosing circle and centroid
            c = max(countours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            moments = cv2.moments(c)
            if moments["m00"] > 0:
                center = int(moments["m10"] / moments["m00"]), \
                         int(moments["m01"] / moments["m00"])
            else:
                center = int(x), int(y)

            # only proceed if the radius meets a minimum size
            laser_msg = laser_location_msg()
            laser_msg.xsize = self.cam_width
            if radius > 1:
                # draw the circle and centroid on the frame,
                cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                laser_msg.detected = True              
                laser_msg.x = int(x)
            else:
                laser_msg = laser_location_msg()                    
                laser_msg.detected = False
                laser_msg.x = 0
            self.target_pub.publish(laser_msg)

    #def detect adapted from "python-laser-tracker" from Brad Montgomery, Aug 2016
    #https://github.com/bradmontgomery/python-laser-tracker
    def detect(self, frame):
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # split the video frame into color channels
        h, s, v = cv2.split(hsv_img)
        self.channels['hue'] = h
        self.channels['saturation'] = s
        self.channels['value'] = v

        # Threshold ranges of HSV components; storing the results in place
        self.threshold_image("hue")
        self.threshold_image("saturation")
        self.threshold_image("value")

        # Perform an AND on HSV components to identify the laser!
        self.channels['laser'] = cv2.bitwise_and(
            self.channels['hue'],
            self.channels['value']
        )
        self.channels['laser'] = cv2.bitwise_and(
            self.channels['saturation'],
            self.channels['laser']
        )

        # Merge the HSV components back together.
        hsv_image = cv2.merge([
            self.channels['hue'],
            self.channels['saturation'],
            self.channels['value'],
        ])

        self.track(frame, self.channels['laser'])

        return hsv_image  
    
    # Adapted from ROS Tutorial Converting between ROS images and OpenCV images (Python)
    # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
    def callback(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        (rows,cols,channels) = cv_image.shape    
        hsv_image = self.detect(cv_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

if __name__ == '__main__':
    rospy.init_node('laser_analyser')
    rospy.loginfo("laser_analyser is now running...")
    image_converter()  
    rospy.spin() 
