#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas

class camera_process:
    global hsv_inferior, hsv_superior
    #hsv_inferior = np.array([100, 50, 50])  # HSV mínimo azul
    #hsv_superior = np.array([140, 255, 255])  # HSV máximo azul
    #hsv_inferior = np.array([0, 50, 50])  # HSV mínimo rojo
    #hsv_superior = np.array([10, 255, 255])  # HSV máximo rojo
    hsv_inferior = np.array([40, 50, 50])  # HSV mínimo verde
    hsv_superior = np.array([80, 255, 255])  # HSV máximo verde


    def camera_callback(self, data):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        cv_image = cv2.resize(cv_image, (320, 240))

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_inferior, hsv_superior)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            cv2.drawContours(cv_image, [contour], -1, (255, 255, 255), 13)
            M = cv2.moments(contour)
            cx,cy= None, None
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
                    self.publicador_centroide.publish(centroide_msg)
        # Mostrar imagen procesada en cv_imagen

        imagen_resized = cv2.resize(cv_image, (600, 600))  # Redimensionar a 500x500 píxeles
        cv2.imshow('Imagen con reconocimiento', imagen_resized)
        cv2.waitKey(1)

########################################################################################################################################################################################################

        

    def __init__(self):
        rospy.init_node('nodo_reconocimiento_forma', anonymous=True)
        self.namespace = rospy.get_param('~namespace', 'default_value') 
        #Ejecución de movimiento del robot
        self.camera_callback = rospy.Subscriber(f"{self.namespace}/usb_cam/image_raw", Image, self.camera_callback)
        self.publicador_centroide = rospy.Publisher(f'{self.namespace}/objeto/Centroide', Point, queue_size=10)
        self.publicador_dimensiones = rospy.Publisher(f'{self.namespace}/dimensiones_imagen', Point, queue_size=10)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()

    def run():
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()

if __name__ == '__main__':
    Process_camera = camera_process()  
    camera_process.run() 
