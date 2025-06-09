#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas

class reconocimiento_forma(object):
    global rojo_min, rojo_max,verde_min,verde_max
    rojo_min = np.array([0, 50, 50])  # HSV mínimo rojo
    rojo_max = np.array([10, 255, 255])  # HSV máximo rojo
    verde_min = np.array([40, 50, 50])  # HSV mínimo verde
    verde_max = np.array([80, 255, 255])  # HSV máximo verde

    
    def camera_callback(self,data):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        cv_image = cv2.resize(cv_image, (320, 240))

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
    # Detectar rojo
        mask_rojo = cv2.inRange(hsv_image, rojo_min, rojo_max)
        contours_rojo, _ = cv2.findContours(mask_rojo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_rojo:
            area = cv2.contourArea(contour)
            if area > 100:  # filtra ruido pequeño
                cv2.drawContours(cv_image, [contour], -1, (0, 0, 255), 2)  # rojo
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, "Rojo", (cx - 20, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    #publicar e topico de centroide 
                    centroide_msg = Point()
                    centroide_msg.x = cx
                    centroide_msg.y = cy
                    centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                    if not rospy.is_shutdown():
                        self.publicador_centroide.publish(centroide_msg)

        # Detectar verde
        mask_verde = cv2.inRange(hsv_image, verde_min, verde_max)
        contours_verde, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_verde:
            area = cv2.contourArea(contour)
            if area > 100:
                cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)  # verde
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.putText(cv_image, "Verde", (cx - 20, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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
    