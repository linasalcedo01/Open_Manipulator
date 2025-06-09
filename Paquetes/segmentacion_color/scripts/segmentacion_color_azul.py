#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Bool

# Variables globales para rangos de color en HSV
global hsv_min, hsv_max
hsv_min = np.array([100, 50, 50])  # HSV mínimo
hsv_max = np.array([140, 255, 255])  # HSV máximo

    
def camera_callback(data):

    global hsv_min, hsv_max
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
        return
    
    cv_image = cv2.resize(cv_image, (320, 240))
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv_image, hsv_min, hsv_max)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        cv2.drawContours(cv_image, [contour], -1, (255, 255, 255), 9)
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
    cv2.imshow('Original', cv_image)
    cv2.waitKey(1)


def main():
    rospy.init_node('nodo_segmentador_Color', anonymous=True)
    rospy.Subscriber("/gamora/usb_cam/image_raw", Image, camera_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()