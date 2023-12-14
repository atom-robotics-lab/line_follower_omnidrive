#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError

class Camera1:

    def __init__(self):
        
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("feedback", Float32, queue_size=10) 
        

    def callback(self, data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        image = cv_image
        
        # Crop the lower half of the image
        resized_image = image[700:800, 130:655]
        

        hsv = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

        # Define range of black color in HSV
        lower_black = np.array([0, 0, 31])
        upper_black = np.array([0,0,51])

        # Threshold the HSV image to get only black colors
        b_mask = cv2.inRange(hsv, lower_black, upper_black)

        # Find contours of black areas
        bluecnts = cv2.findContours(b_mask.copy(),
                                    cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(bluecnts) > 0:
            black_area = max(bluecnts, key=cv2.contourArea)
            (xg, yg, wg, hg) = cv2.boundingRect(black_area)

            # Calculate the center of the bounding box
            x_bb_center = xg + wg / 2
            #y_bb_center = yg + hg / 2

            # Calculate the center of the camera window
            camera_width, camera_height = resized_image.shape[1], resized_image.shape[0]
            x_camera_center = camera_width / 2
            #y_camera_center = camera_height / 2

            # Calculate the distance between the centers
            #distance_to_center = math.sqrt((x_bb_center - x_camera_center)**2 + (y_bb_center - y_camera_center)**2)
            distance_to_center= (x_bb_center-x_camera_center)
            self.pub.publish(distance_to_center)

            # Print the distance to the console
            rospy.loginfo(f"Distance between bounding box center and camera center: {distance_to_center} pixels")

            # Draw the bounding box
            cv2.rectangle(resized_image, (xg, yg), (xg + wg, yg + hg), (0, 255, 0), 2)

        # Display the images
        cv2.imshow("Camera output resized", resized_image)
        cv2.imshow("mask", b_mask)

        cv2.waitKey(3)

def main():
    Camera1()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()