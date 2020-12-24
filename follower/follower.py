#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import cv_bridge


# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

image_input = 0
MIN_AREA = 10000
error = 0
LINEAR = 0.2
KP = 1.5/100
just_seen = False

def crop_size(height, width):
    return (1*height//3, height, width//4, 3*width//4)

def image_callback(msg):
    """Function to be called whenever a new Image message arrives"""
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

def get_contour_centroid(mask, out):
    """
    Return the centroid of the largest contour in the binary image 'mask'
    and draw the contours on 'out'
    """ 
    # get a list of contours
    contours, h = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    largest_area = 0
    centroid = {}

    for contour in contours:
        cv2.drawContours(out, contour, -1, (255,0,0), 1) 
        # Print the contour area in blue
        
        M = cv2.moments(contour)
        # Search more about Image Moments on Wikipedia :)

        if M['m00'] > 0:
        # if countor.area > 0:

            # print the area    
            cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0), 2)

            if (M['m00'] > MIN_AREA):
                largest_area = M['m00']
                centroid['x'] = crop_w_start + int(M["m10"]/M["m00"])
                centroid['y'] = int(M["m01"]/M["m00"])

    return centroid

def timer_callback():
    global error

    # Wait for the first image to be received
    while type(image_input) != np.ndarray:
        pass
    
    height, width, _ = image_input.shape

    while rclpy.ok():

        image = image_input.copy()

        global crop_w_start
        crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

        # get the bottom part of the image (matrix slicing)
        crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
        
        # get a binary picture, where non-zero values represent the line.
        mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

        cv2.imshow('mask', mask)

        # get the centroid of the biggest contour in the picture,
        # and plot its detail on the bottom part of the output image
        output = image
        centroid = get_contour_centroid(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])  

        # error: The difference between the center of the camera
        # and the center of the line

        message = Twist()
        
        if centroid:
            # if there even is a centroid:
            # (as the camera could not be reading any lines)      
            x = centroid['x']
            error = x - width//2
            print("Error:{}".format(error))
            message.linear.x = LINEAR
            just_seen = True
        else:
            if just_seen:
                just_seen = False
                error = error * 1.2
            message.linear.x = 0


        # Create an empty Twist message, then give it values
        # THESE ARE THE CONTROL VALUES TO BE CHANGED AND TESTED
        message.angular.z = float(error) * -KP
        print("Linear Z: {}".format(message.linear.z))

        # Plot the boundaries where the image was cropped
        cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)

        if centroid:
            # If there is a line to be followed, print its centroid
            cv2.circle(output, (centroid['x'], crop_h_start + centroid['y']), 5, (0,255,0), 7)

        # Uncomment to print the binary picture
        #cv2.imshow("mask", mask)

        # Show the output image to the user
        cv2.imshow("output", output)
        # Print the image for 5milis, then resume the execution
        cv2.waitKey(5)

        # Publish the message to 'cmd_vel'
        publisher.publish(message)


# BGR values to filter only the color of the line
lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([238, 142, 158])

rclpy.init()
node = Node('follower')
publisher = node.create_publisher(Twist, 'cmd_vel', 3)

subscription = node.create_subscription(
        Image, 'camera/image_raw', image_callback, 10)

timer_period = 0.06 # seconds
publisher_timer = node.create_timer(timer_period, timer_callback)

def main():
    rclpy.spin(node)

try:
    main()
except rclpy.exceptions.ROSInterruptException:
    node.destroy_node()
    rclpy.shutdown()
