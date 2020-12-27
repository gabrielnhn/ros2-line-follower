#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import cv2
import cv_bridge

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

## User-defined macros:
# Minimum size for a contour to be considered
MIN_AREA = 10000 
# Robot's speed when following the line
LINEAR_SPEED = 0.2
# Proportional constant to be applied on speed when turning 
# (Multiplied by the error)
KP = 1.5/100 

# Global vars. initial values
image_input = 0
error = 0
just_seen = False

def crop_size(height, width):
    """Get the measures to crop the image"""
    ## Update these values to your liking.

    return (1*height//3, height, width//4, 3*width//4)

    # Height_upper_boundary, Height_lower_boundary,
        # Width_left_boundary, Width_right_boundary

def image_callback(msg):
    """
    Function to be called whenever a new Image message arrives.
    Update the global variable 'image_input'
    """
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    # node.get_logger().info('Received image')

def get_contour_centroid(mask, out):
    """
    Return the centroid of the largest contour in the binary image 'mask'
    (If there is a contour),
    and draw all contours on 'out' image
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
    """
    Function to be called when the timer ticks.
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    global error
    global image_input
    global just_seen # a contour

    # Wait for the first image to be received
    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape

    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    
    # get a binary picture, where non-zero values represent the line.
    # (filter the color values so only the contour is seen)
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    # get the centroid of the biggest contour in the picture,
    # and plot its detail on the cropped part of the output image
    output = image
    centroid = get_contour_centroid(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])  

    # Create an empty Twist message, then give it values
    message = Twist()
    
    if centroid:
        # if there even is a centroid:
        # (as the camera could not be reading any lines)      
        x = centroid['x']

        # error:= The difference between the center of the camera
        # and the center of the line
        error = x - width//2
        print("Error:{}".format(error))

        message.linear.x = LINEAR_SPEED
        just_seen = True

    else:
        # There is no line in the image. 
        # Turn on the spot to find it again. 
        if just_seen:
            just_seen = False
            error = error * 1.2
        message.linear.x = 0.0

    # Determine the speed to turn and get the line in the center of the camera.
    message.angular.z = float(error) * -KP
    print("Angular Z: {}".format(message.angular.z))

    # Plot the boundaries where the image was cropped
    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)

    if centroid:
        # plot the centroid on the image
        cv2.circle(output, (centroid['x'], crop_h_start + centroid['y']), 5, (0,255,0), 7)

    # Uncomment to show the binary picture
    #cv2.imshow("mask", mask)

    # Show the output image to the user
    cv2.imshow("output", output)
    # Print the image for 5milis, then resume execution
    cv2.waitKey(5)

    # Publish the message to 'cmd_vel'
    publisher.publish(message)


# BGR values to filter only the color of the line
lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([238, 142, 158])

def main():
    rclpy.init()
    global node
    node = Node('follower')
    global publisher
    publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
    subscription = node.create_subscription(Image, 'camera/image_raw',
                                            image_callback,
                                            rclpy.qos.qos_profile_sensor_data)
    timer_period = 0.06 # seconds
    timer = node.create_timer(timer_period, timer_callback)
    rclpy.spin(node)

try:
    main()
except rclpy.exceptions.ROSInterruptException:
    node.destroy_node()
    rclpy.shutdown()
