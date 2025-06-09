#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas

class reconocimiento_forma(object):

    def camera_callback(self,data):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        cv_image = cv2.resize(cv_image, (320, 240))

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('gray_image', gray_image)
        canny_min = 10
        canny_max=150
        canny=cv2.Canny(gray_image,canny_min,canny_max)
        cv2.imshow('canny', canny)
        # Encontrar y dibujar contornos
        contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        

        
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            if len(approx) == 8:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)  # Dibujar centroide
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 255), 9)
                    #publicar e topico de centroide 
                    centroide_msg = Point()
                    centroide_msg.x = cx
                    centroide_msg.y = cy
                    centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                    if not rospy.is_shutdown():
                        self.publicador_centroide.publish(centroide_msg)

        cv2.imshow("Shape Detection", cv_image)
        cv2.waitKey(1)
        


    def __init__(self):
        rospy.init_node('segmentador_objetos_forma', anonymous=True)
        rospy.Subscriber("/gamora/usb_cam/image_raw", Image, self.camera_callback)
        self.publicador_centroide = rospy.Publisher('/gamora/objeto/Centroide', Point, queue_size=10)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = reconocimiento_forma()
    

