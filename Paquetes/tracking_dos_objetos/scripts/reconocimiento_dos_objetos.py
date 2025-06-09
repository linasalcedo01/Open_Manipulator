#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas

class camera_process():
    global tipo_reconocimiento
    tipo_reconocimiento= 'color'

    #SI ELIGES RECONOCER OBJETOS DE DIFERENTE COLOR ESCRIBE SUS RANGOS
    global objeto1_min, objeto1_max, objeto2_min, objeto2_max
    objeto1_min = np.array([0, 50, 50])  # HSV mínimo rojo
    objeto1_max = np.array([10, 255, 255])  # HSV máximo rojo
    objeto2_min = np.array([40, 50, 50])  # HSV mínimo verde
    objeto2_max = np.array([80, 255, 255])  # HSV máximo verde

    #SI ELIGES RECONOCER OBJETOS DE DIFERENTE FORMA ESCRIBE SUS LADOS
    global objeto1_lados, objeto2_lados
    objeto1_lados = 8
    objeto2_lados = 4



    def camera_callback(self, data):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        except CvBridgeError as e:
            print(e)
            return
        cv_image = cv2.resize(cv_image, (320, 240))

        #Tipo de reconocimiento
        global tipo_reconocimiento
        if tipo_reconocimiento is None:
            self.estate= 'Imagen sin reconocimiento'
            print("sin reconocimiento")
        if tipo_reconocimiento=='color':
            self.estate = 'Reconocimiento de dos objetos de diferente COLOR'
            global objeto1_min, objeto1_max, objeto2_min, objeto2_max
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Objeto1
            mask_objeto1 = cv2.inRange(hsv_image, objeto1_min, objeto1_max)
            contours_objeto1, _ = cv2.findContours(mask_objeto1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours_objeto1:
                area = cv2.contourArea(contour)
                if area > 100:  # filtra ruido pequeño
                    cv2.drawContours(cv_image, [contour], -1, (0, 0, 255), 2)  # rojo
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                        #publicar e topico de centroide 
                        centroide_msg = Point()
                        centroide_msg.x = cx
                        centroide_msg.y = cy
                        centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                        if not rospy.is_shutdown():
                            self.publicador_centroide_objeto1.publish(centroide_msg)

        # Objeto2
            mask_objeto2 = cv2.inRange(hsv_image, objeto2_min, objeto2_max)
            contours_objeto2, _ = cv2.findContours(mask_objeto2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours_objeto2:
                area = cv2.contourArea(contour)
                if area > 100:
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)  # verde
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                        #publicar e topico de centroide 
                        centroide_msg = Point()
                        centroide_msg.x = cx
                        centroide_msg.y = cy
                        centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                        if not rospy.is_shutdown():
                            self.publicador_centroide_objeto2.publish(centroide_msg)

        if tipo_reconocimiento=='forma':
            self.estate = 'Reconocimiento de dos objetos de diferente FORMA'
            global objeto1_lados, objeto2_lados  

        #Objeto1
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            canny_min = 10
            canny_max=150
            canny=cv2.Canny(gray_image,canny_min,canny_max)
            contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                if len(approx) == objeto1_lados:
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
                            self.publicador_centroide_objeto1.publish(centroide_msg)
        #Objeto2
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            canny_min = 10
            canny_max=150
            canny=cv2.Canny(gray_image,canny_min,canny_max)
            contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                if len(approx) == objeto2_lados:
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
                            self.publicador_centroide_objeto2.publish(centroide_msg)
                        

#        cv_image = cv2.resize(cv_image, (600, 600))
        cv2.imshow(self.estate, cv_image)
        cv2.waitKey(1)
                    
###################################################################################################################
    def __init__(self):
        rospy.init_node('nodo_reconocimiento_forma', anonymous=True)
        self.camera_callback = rospy.Subscriber("/gamora/usb_cam/image_raw", Image, self.camera_callback)
        self.publicador_centroide_objeto1= rospy.Publisher('/gamora/objeto1/Centroide', Point, queue_size=10)
        self.publicador_centroide_objeto2 = rospy.Publisher('/gamora/objeto2/Centroide', Point, queue_size=10)

    def run():
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()

if __name__ == '__main__':
    try:
        Process_camera = camera_process()  
        camera_process.run() 
    except rospy.ROSInterruptException:
        pass
