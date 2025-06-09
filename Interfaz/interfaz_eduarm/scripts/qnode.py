#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError
import cv2  # OpenCV
import rospy
import time
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas
import roslaunch
import subprocess, os, signal
from sensor_msgs.msg import JointState, Image
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest
)
from open_manipulator_msgs.msg import JointPosition, KinematicsPose, OpenManipulatorState
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap


class QNode(QObject):
    joint_state_signal = pyqtSignal(list)
    kinematics_pose_signal = pyqtSignal(list)

    def __init__(self):
        super(QNode, self).__init__()
        rospy.init_node('qnode_joint_state_subscriber', anonymous=True)
        global ARREGLO
        ARREGLO = None
        global HSV_inferior
        global HSV_superior
        global canny_min
        global canny_max
        global vertices
        HSV_inferior = None  
        HSV_superior = None  
        canny_min =None
        canny_max=None
        vertices=None
        self.camarareturn2 = None
        self.camarareturn5 = None
        
        self.present_kinematic_position = [0.0, 0.0, 0.0]
        self.kinematics_pose = None
        self.open_manipulator_is_moving = False
        self.open_manipulator_actuator_enabled = False
        self.process_sim = None  # Para el proceso de la simulación
        self.process_real = None  # Para el proceso real
        self.eleccion_hostname_robot=None
        self.autorizacion_hostname=None
        self.camarareturn = None


 
    def seleccion_hostname(self, seleccion):
        self.hostname_seleccionado=seleccion
    

    def hostname_real(self):
        if self.hostname_seleccionado == '/nebula':
            self.nebula_joint_state_subscriber = rospy.Subscriber('/nebula/joint_states', JointState, self.joint_states_callback)
            self.nebula_kinematics_pose_subscriber = rospy.Subscriber('/nebula/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
            self.nebula_open_manipulator_states_sub = rospy.Subscriber("/nebula/states", OpenManipulatorState, self.manipulator_states_callback)
            self.nebula_camera_manipulator = rospy.Subscriber('/nebula/usb_cam/image_raw', Image, self.camera_manipulator_Callback)
            self.nebula_camera_reco_color = rospy.Subscriber('/nebula/usb_cam/image_raw', Image, self.camera_reco_color_callback)
            self.nebula_camera_reco_forma = rospy.Subscriber('/nebula/usb_cam/image_raw', Image, self.camera_reco_forma_callback)
            self.nebula_camera_manipulator_real = rospy.Subscriber('/nebula/usb_cam/image_raw', Image, self.camera_manipulator_Callback_real)
            self.nebula_camera_reco_color_real = rospy.Subscriber('/nebula/usb_cam/image_raw', Image, self.camera_reco_color_callback_real)
            self.nebula_camera_reco_forma_real = rospy.Subscriber('/nebula/usb_cam/image_raw', Image, self.camera_reco_forma_callback_real)
            self.nebula_publicador_centroide = rospy.Publisher('/nebula/objeto/Centroide', Point, queue_size=10)
            self.nebula_publicador_dimensiones= rospy.Publisher('/nebula/dimensiones_imagen', Point, queue_size=10)

            self.nebula_set_actuator_state_client = rospy.ServiceProxy('/nebula/set_actuator_state', SetActuatorState)
            self.nebula_set_joint_position_client = rospy.ServiceProxy('/nebula/goal_joint_space_path', SetJointPosition)
            self.nebula_set_tool_control_client = rospy.ServiceProxy('/nebula/goal_tool_control', SetJointPosition)
            self.nebula_goal_task_space_path_position_only_client = rospy.ServiceProxy('/nebula/goal_task_space_path_position_only', SetKinematicsPose)


        if self.hostname_seleccionado =='/gamora':
            self.gamora_joint_state_subscriber = rospy.Subscriber('/gamora/joint_states', JointState, self.joint_states_callback)
            self.gamora_kinematics_pose_subscriber = rospy.Subscriber('/gamora/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
            self.gamora_open_manipulator_states_sub = rospy.Subscriber("/gamora/states", OpenManipulatorState, self.manipulator_states_callback)
            self.gamora_camera_manipulator = rospy.Subscriber('/gamora/usb_cam/image_raw', Image, self.camera_manipulator_Callback)
            self.gamora_camera_reco_color = rospy.Subscriber('/gamora/usb_cam/image_raw', Image, self.camera_reco_color_callback)
            self.gamora_camera_reco_forma = rospy.Subscriber('/gamora/usb_cam/image_raw', Image, self.camera_reco_forma_callback)
            self.gamora_camera_manipulator_real = rospy.Subscriber('/gamora/usb_cam/image_raw', Image, self.camera_manipulator_Callback_real)
            self.gamora_camera_reco_color_real = rospy.Subscriber('/gamora/usb_cam/image_raw', Image, self.camera_reco_color_callback_real)
            self.gamora_camera_reco_forma_real = rospy.Subscriber('/gamora/usb_cam/image_raw', Image, self.camera_reco_forma_callback_real)
            self.gamora_publicador_centroide = rospy.Publisher('/gamora/objeto/Centroide', Point, queue_size=10)
            self.gamora_publicador_dimensiones= rospy.Publisher('/gamora/dimensiones_imagen', Point, queue_size=10)

            self.gamora_set_actuator_state_client = rospy.ServiceProxy('/gamora/set_actuator_state', SetActuatorState)
            self.gamora_set_joint_position_client = rospy.ServiceProxy('/gamora/goal_joint_space_path', SetJointPosition)
            self.gamora_set_tool_control_client = rospy.ServiceProxy('/gamora/goal_tool_control', SetJointPosition)
            self.gamora_goal_task_space_path_position_only_client = rospy.ServiceProxy('/gamora/goal_task_space_path_position_only', SetKinematicsPose)

    def desconectar_hostname_real(self):
        if self.hostname_seleccionado == '/nebula':
            # Lista de suscriptores
            subs = [
                'nebula_joint_state_subscriber',
                'nebula_kinematics_pose_subscriber',
                'nebula_open_manipulator_states_sub',
                'nebula_camera_manipulator',
                'nebula_camera_reco_color',
                'nebula_camera_reco_forma',
                'nebula_camera_manipulator_real',
                'nebula_camera_reco_color_real',
                'nebula_camera_reco_forma_real']
            for sub in subs:
                if hasattr(self, sub):
                    getattr(self, sub).unregister()
                    setattr(self, sub, None)
            # Lista de publicadores
            pubs = [
                'nebula_publicador_centroide',
                'nebula_publicador_dimensiones']
            for pub in pubs:
                if hasattr(self, pub):
                    try:
                        getattr(self, pub).unregister()
                    except Exception as e:
                        rospy.logwarn(f"No se pudo cerrar publisher {pub}: {e}")
                    setattr(self, pub, None)
            # Lista de proxies de servicios
            services = [
                'nebula_set_actuator_state_client',
                'nebula_set_joint_position_client',
                'nebula_set_tool_control_client',
                'nebula_goal_task_space_path_position_only_client']
            for srv in services:
                if hasattr(self, srv):
                    delattr(self, srv)
            rospy.loginfo("✅ Todos los tópicos, publicadores y servicios de Nebula han sido detenidos.")

        if self.hostname_seleccionado == '/gamora':
                    # Lista de suscriptores
            subs = [
                'gamora_joint_state_subscriber',
                'gamora_kinematics_pose_subscriber',
                'gamora_open_manipulator_states_sub',
                'gamora_camera_manipulator',
                'gamora_camera_reco_color',
                'gamora_camera_reco_forma',
                'gamora_camera_manipulator_real',
                'gamora_camera_reco_color_real',
                'gamora_camera_reco_forma_real']
            for sub in subs:
                if hasattr(self, sub):
                    getattr(self, sub).unregister()
                    setattr(self, sub, None)
            # Lista de publishers
            pubs = [
                'gamora_publicador_centroide',
                'gamora_publicador_dimensiones']
            for pub in pubs:
                if hasattr(self, pub):
                    try:
                        getattr(self, pub).unregister()
                    except Exception as e:
                        rospy.logwarn(f"No se pudo cerrar publisher {pub}: {e}")
                    setattr(self, pub, None)
            # Lista de proxies de servicios
            services = [
                'gamora_set_actuator_state_client',
                'gamora_set_joint_position_client',
                'gamora_set_tool_control_client',
                'gamora_goal_task_space_path_position_only_client']
            for srv in services:
                if hasattr(self, srv):
                    delattr(self, srv)
            rospy.loginfo("✅ Todos los tópicos, publicadores y servicios de Gamora han sido detenidos.")

    def desconectar_hostname_sim(self):
        # Lista de suscriptores
        self.present_joint_angle=None
        subs = [
            'joint_state_subscriber',
            'kinematics_pose_subscriber',
            'open_manipulator_states_sub',
            'camera_manipulator',
            'camera_reco_color',
            'camera_reco_forma',
            'camera_manipulator_real',
            'camera_reco_color_real',
            'camera_reco_forma_real']
        for sub in subs:
            if hasattr(self, sub):
                getattr(self, sub).unregister()
                setattr(self, sub, None)
        # Lista de publicadores
        pubs = [
            'publicador_centroide',
            'publicador_dimensiones']
        for pub in pubs:
            if hasattr(self, pub):
                try:
                    getattr(self, pub).unregister()
                except Exception as e:
                    rospy.logwarn(f"No se pudo cerrar publisher {pub}: {e}")
                setattr(self, pub, None)
        # Lista de proxies de servicios
        services = [
            'set_actuator_state_client',
            'set_joint_position_client',
            'set_tool_control_client',
            'goal_task_space_path_position_only_client']
        for srv in services:
            if hasattr(self, srv):
                delattr(self, srv)

        rospy.loginfo("✅ Todos los tópicos, publicadores y servicios de Nebula han sido detenidos.")



    def hostname_simulado(self):
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.kinematics_pose_subscriber = rospy.Subscriber('/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.open_manipulator_states_sub = rospy.Subscriber("/states", OpenManipulatorState, self.manipulator_states_callback)
        self.camera_manipulator = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_manipulator_Callback)
        self.camera_reco_color = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_reco_color_callback)
        self.camera_reco_forma = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_reco_forma_callback)
        self.camera_manipulator_real = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_manipulator_Callback_real)
        self.camera_reco_color_real = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_reco_color_callback_real)
        self.camera_reco_forma_real = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_reco_forma_callback_real)
        self.publicador_centroide = rospy.Publisher('/objeto/Centroide', Point, queue_size=10)
        self.publicador_dimensiones= rospy.Publisher('/dimensiones_imagen', Point, queue_size=10)

        self.set_actuator_state_client = rospy.ServiceProxy('/set_actuator_state', SetActuatorState)
        self.set_joint_position_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        self.set_tool_control_client = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy('/goal_task_space_path_position_only', SetKinematicsPose)

        

    

    
    def set_reco_color(self, hsv_inferior,hsv_superior): #estoy cambiando la variable global DATA por el arreglo data que me llega
        global HSV_inferior
        global HSV_superior
        HSV_inferior = hsv_inferior
        HSV_superior = hsv_superior
        print(f"este es el que llega en Qnode process HSV inferior {HSV_inferior} y HSV superior {HSV_superior}")

    def set_reco_forma(self, canny_inferior, canny_superior, cantidad_vertices): #estoy cambiando la variable global DATA por el arreglo data que me llega
            global canny_min
            global canny_max
            global vertices
            canny_min = canny_inferior
            canny_max = canny_superior
            vertices = cantidad_vertices
            print(f"este es el que llega en Qnode de canny inf {canny_min} y el mas {canny_max} y el {vertices} ")

    def set_reset_data(self):
        global HSV_inferior
        global HSV_superior
        HSV_inferior = None
        HSV_superior = None
        print(f"en Qnode es en reset {HSV_inferior} y {HSV_superior}")
        return HSV_inferior , HSV_superior

    def set_reset_data_forma(self):
        global canny_min
        global canny_max
        global vertices
        canny_min =None
        canny_max = None
        vertices = None
        return canny_min, canny_max, vertices
    
    def set_reset_data_real(self):
        global HSV_inferior
        global HSV_superior
        HSV_inferior = None
        HSV_superior = None
        print(f"en Qnode es en reset {HSV_inferior} y {HSV_superior}")
        return HSV_inferior , HSV_superior

    def set_reset_data_forma_real(self):
        global canny_min
        global canny_max
        global vertices
        canny_min =None
        canny_max = None
        vertices = None
        return canny_min, canny_max, vertices
###################este es mi PROCESSSSSSSSSSSSSSSSSSSSSSSSSS##############################################################################################################3

    def camera_reco_color_callback(self,msg): #este hay que modificarlo para el procesamiento del color
        """Callback para recibir imagen de ROS, y se procesa con CvBridge para recibir la imagen y almacenarla en Cv_image"""
        bridge = CvBridge()
        try:
            cv_imagen = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        global HSV_inferior
        global HSV_superior

        hsv_inferior = HSV_inferior
        hsv_superior = HSV_superior
        if HSV_inferior is None and HSV_superior is None:
            self.camarareturn2=False
        if hsv_inferior is not None and hsv_superior is not None:
            hsv_imagen = cv2.cvtColor(cv_imagen, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_imagen, hsv_inferior, hsv_superior)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                cv2.drawContours(cv_imagen, [contour], -1, (255, 255, 255), 13)
                M = cv2.moments(contour)
                cx,cy= None, None
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(cv_imagen, (cx, cy), 5, (0, 255, 0), -1)
                    #publicar e topico de centroide 
                    centroide_msg = Point()
                    centroide_msg.x = cx
                    centroide_msg.y = cy
                    centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                    if not rospy.is_shutdown():
                        self.publicador_centroide.publish(centroide_msg)
            # Mostrar imagen procesada en cv_imagen
            height, width, channel = cv_imagen.shape
            bytes_per_line = 3 * width
            cv_imagen = cv2.cvtColor(cv_imagen, cv2.COLOR_BGR2RGB)
            q_image = QImage(cv_imagen.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap2 = QPixmap.fromImage(q_image)
            self.camarareturn2 = pixmap2


    def set_image(self):
        "Envía el pixmap a interfaz"
        if self.camarareturn2 is not False:
            return self.camarareturn2
        else:
            return False
        
        
    def camera_reco_color_callback_real(self,msg): #este hay que modificarlo para el procesamiento del color
        """Callback para recibir imagen de ROS, y se procesa con CvBridge para recibir la imagen y almacenarla en Cv_image"""
        bridge = CvBridge()
        try:
            cv_imagen = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        global HSV_inferior
        global HSV_superior

        hsv_inferior = HSV_inferior
        hsv_superior = HSV_superior
        if HSV_inferior is None and HSV_superior is None:
            self.camarareturn8=False
        if hsv_inferior is not None and hsv_superior is not None:
            hsv_imagen = cv2.cvtColor(cv_imagen, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_imagen, hsv_inferior, hsv_superior)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                cv2.drawContours(cv_imagen, [contour], -1, (255, 255, 255), 13)
                M = cv2.moments(contour)
                cx,cy= None, None
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(cv_imagen, (cx, cy), 5, (0, 255, 0), -1)
                    #publicar e topico de centroide 
                    centroide_msg = Point()
                    centroide_msg.x = cx
                    centroide_msg.y = cy
                    centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                    if not rospy.is_shutdown():
                        self.publicador_centroide.publish(centroide_msg)
            # Mostrar imagen procesada en cv_imagen
            height, width, channel = cv_imagen.shape
            bytes_per_line = 3 * width
            cv_imagen = cv2.cvtColor(cv_imagen, cv2.COLOR_BGR2RGB)
            q_image = QImage(cv_imagen.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap8 = QPixmap.fromImage(q_image)
            self.camarareturn8 = pixmap8


    def set_image_real(self):
        "Envía el pixmap a interfaz"
        if self.camarareturn8 is not False:
            return self.camarareturn8
        else:
            return False
##########################################################################################################################################################3
    
    def camera_reco_forma_callback(self,msg):
        """Callback para recibir imagen de ROS, y se procesa con CvBridge para recibir la imagen y almacenarla en Cv_image"""
        bridge = CvBridge()
        try:
            cv_imagen2 = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        global canny_min
        global canny_max
        global vertices
        canny_inferior = canny_min
        canny_superior = canny_max
        cantidad_vertices = vertices
        if canny_inferior is None and canny_superior is None:
            self.camarareturn5=False
        if canny_inferior is not None and canny_superior is not None:
            gray_image = cv2.cvtColor(cv_imagen2, cv2.COLOR_BGR2GRAY)
            canny=cv2.Canny(gray_image,canny_min,canny_max)
            # Encontrar y dibujar contornos
            
            contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                if len(approx) == cantidad_vertices:
                    M = cv2.moments(contour)
                    cx,cy= None, None
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    cv2.drawContours(cv_imagen2, contours, -1, (255, 255, 255), 13)
                    cv2.circle(cv_imagen2, (cx, cy), 5, (0, 255, 0), -1)  # Dibujar centroide
                    #publicar e topico de centroide 
                    centroide_msg = Point()
                    centroide_msg.x = cx
                    centroide_msg.y = cy
                    centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                    if not rospy.is_shutdown():
                        self.publicador_centroide.publish(centroide_msg)
            # Mostrar imagen procesada en cv_imagen
            height, width, channel = cv_imagen2.shape
            bytes_per_line = 3 * width
            cv_imagen2 = cv2.cvtColor(cv_imagen2, cv2.COLOR_BGR2RGB)
            q_image = QImage(cv_imagen2.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap5 = QPixmap.fromImage(q_image)
            self.camarareturn5 = pixmap5

                
    def set_image_forma(self):
        "Envía el pixmap a interfaz"
        if self.camarareturn5 is not False:
            return self.camarareturn5
        else:
            return False
        

    def camera_reco_forma_callback_real(self,msg):
        """Callback para recibir imagen de ROS, y se procesa con CvBridge para recibir la imagen y almacenarla en Cv_image"""
        bridge = CvBridge()
        try:
            cv_imagen2 = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        global canny_min
        global canny_max
        global vertices
        canny_inferior = canny_min
        canny_superior = canny_max
        cantidad_vertices = vertices
        if canny_inferior is None and canny_superior is None:
            self.camarareturn9=False
        if canny_inferior is not None and canny_superior is not None:
            gray_image = cv2.cvtColor(cv_imagen2, cv2.COLOR_BGR2GRAY)
            canny=cv2.Canny(gray_image,canny_min,canny_max)
            # Encontrar y dibujar contornos
            
            contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                if len(approx) == cantidad_vertices:
                    M = cv2.moments(contour)
                    cx,cy= None, None
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.drawContours(cv_imagen2, contours, -1, (255, 255, 255), 13)
                    cv2.circle(cv_imagen2, (cx, cy), 5, (0, 255, 0), -1)  # Dibujar centroide
                    #publicar e topico de centroide 
                    centroide_msg = Point()
                    centroide_msg.x = cx
                    centroide_msg.y = cy
                    centroide_msg.z = 0  # Si no hay profundidad, deja en 0
                    if not rospy.is_shutdown():
                        self.publicador_centroide.publish(centroide_msg)
            # Mostrar imagen procesada en cv_imagen
            height, width, channel = cv_imagen2.shape
            bytes_per_line = 3 * width
            cv_imagen2 = cv2.cvtColor(cv_imagen2, cv2.COLOR_BGR2RGB)
            q_image = QImage(cv_imagen2.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap9 = QPixmap.fromImage(q_image)
            self.camarareturn9 = pixmap9

                
    def set_image_forma_real(self):
        "Envía el pixmap a interfaz"
        if self.camarareturn9 is not False:
            return self.camarareturn9
        else:
            return False
        
#####################################################################################################################################################################################
    def camera_manipulator_Callback(self,msg):
        """Callback para recibir imagen de ROS, y se procesa con CvBridge para recibir la imagen y almacenarla en Cv_image"""
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Publicar las dimensiones de píxeles de la imagen
        alto, ancho, _ = cv_image.shape  # Obtener dimensiones
        dimensiones_msg = Point()
        dimensiones_msg.x = int(ancho)  
        dimensiones_msg.y = int(alto)  
        if not rospy.is_shutdown():
            self.publicador_dimensiones.publish(dimensiones_msg)
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.camarareturn = pixmap
    
    def SetCameraManipulator(self, enableCam):
        "Envía el pixmap a interfaz"
        return self.camarareturn
        
    def camera_manipulator_Callback_real(self,msg):
        """Callback para recibir imagen de ROS, y se procesa con CvBridge para recibir la imagen y almacenarla en Cv_image"""
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Publicar las dimensiones de píxeles de la imagen
        alto, ancho, _ = cv_image.shape  # Obtener dimensiones
        dimensiones_msg = Point()
        dimensiones_msg.x = int(ancho)  
        dimensiones_msg.y = int(alto)  
        if not rospy.is_shutdown():
            self.publicador_dimensiones.publish(dimensiones_msg)
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap7 = QPixmap.fromImage(q_image)
        self.camarareturn7 = pixmap7

    def SetCameraManipulator_real(self, enableCam):
        "Envía el pixmap a interfaz"
        return self.camarareturn7




    def joint_states_callback(self, msg):
        """
        Callback para recibir mensajes de /joint_states.
        Extrae las posiciones de las articulaciones y actualiza el estado.
        """
        temp_angle = [0.0] * 5
        
        for i in range(len(msg.name)):
            if msg.name[i] == "joint1":
                temp_angle[0] = msg.position[i]
            elif msg.name[i] == "joint2":
                temp_angle[1] = msg.position[i]
            elif msg.name[i] == "joint3":
                temp_angle[2] = msg.position[i]
            elif msg.name[i] == "joint4":
                temp_angle[3] = msg.position[i]
            elif msg.name[i] == "gripper":
                temp_angle[4] = msg.position[i]

        self.present_joint_angle = temp_angle

    def getPresentJointAngle(self):
        """ Retorna los ángulos actuales de las articulaciones. """
        return self.present_joint_angle

    def kinematics_pose_callback(self, msg):
        """
        Callback para recibir mensajes de KinematicsPose.
        Extrae la posición y actualiza el estado.
        """
        temp_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.present_kinematic_position = temp_position
        self.kinematics_pose = msg.pose

    def getPresentKinematicsPose(self):
        """ Retorna la posición cinemática actual. """
        return self.present_kinematic_position

    def setActuatorState(self, actuator_state):
        """
        Activa o desactiva los actuadores del manipulador.
        :param actuator_state: True para habilitar, False para deshabilitar.
        :return: True si fue exitoso, False si falló.
        """
        try:
            srv = SetActuatorStateRequest()
            srv.set_actuator_state = actuator_state
            response = self.set_actuator_state_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
    def manipulator_states_callback(self, msg):
        """
        Callback para recibir el estado del manipulador.
        """
        self.open_manipulator_is_moving = msg.open_manipulator_moving_state == msg.IS_MOVING
        self.open_manipulator_actuator_enabled = msg.open_manipulator_actuator_state == msg.ACTUATOR_ENABLED

    def get_open_manipulator_moving_state(self):
        """ Retorna si el manipulador está en movimiento. """
        return self.open_manipulator_is_moving

    def get_open_manipulator_actuator_state(self):
        """ Retorna si el actuador del manipulador está habilitado. """
        return self.open_manipulator_actuator_enabled

    def setJointSpacePath(self, joint_name, joint_angle, path_time):
        """
        Envía una solicitud para mover el manipulador en el espacio articular.
        :param joint_name: Lista con nombres de articulaciones.
        :param joint_angle: Lista con ángulos de destino.
        :param path_time: Tiempo de ejecución.
        :return: True si el servicio fue exitoso, False si falló.
        """
        try:
            srv = SetJointPositionRequest()
            srv.joint_position.joint_name = joint_name
            srv.joint_position.position = joint_angle
            srv.path_time = path_time
            response = self.set_joint_position_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def setToolControl(self, joint_angle):
        """
        Controla la herramienta (gripper).
        :param joint_angle: Lista con la posición deseada del gripper.
        :return: True si el servicio fue exitoso, False si falló.
        """
        try:
            srv = SetJointPositionRequest()
            srv.joint_position.joint_name.append("gripper")
            srv.joint_position.position = joint_angle
            response = self.set_tool_control_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def setTaskSpacePath(self, kinematics_pose, path_time):
        """
        Envía una solicitud para mover el efector final en el espacio cartesiano.
        :param kinematics_pose: Lista con las coordenadas [x, y, z] de destino.
        :param path_time: Tiempo de ejecución.
        :return: True si la planificación fue exitosa, False si falló.
        """
        try:
            srv = SetKinematicsPoseRequest()
            srv.end_effector_name = "gripper"
            srv.kinematics_pose.pose.position.x = kinematics_pose[0]
            srv.kinematics_pose.pose.position.y = kinematics_pose[1]
            srv.kinematics_pose.pose.position.z = kinematics_pose[2]

            # Mantener la orientación actual del efector final
            srv.kinematics_pose.pose.orientation = self.kinematics_pose.orientation

            srv.path_time = path_time
            response = self.goal_task_space_path_position_only_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False


    def start_robot_controller_sim(self):
        """Inicia el nodo del controlador del manipulador."""

        try:
            self.process = subprocess.Popen(
                ["roslaunch", "open_manipulator_controller", "open_manipulator_controller.launch", "use_platform:=false"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Importante para poder terminarlo después
            )
        except Exception as e:
            rospy.logerr(f"Error al lanzar el nodo: {e}")

    def stop_robot_controller_sim(self):
        self.desconectar_hostname_sim()
        
        """Detiene el nodo del controlador del manipulador."""
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)  # Terminar el grupo de procesos
            self.process.wait()  # Esperar a que termine
            self.process = None
            print("Nodo del controlador detenido.")
        else:
            print("No hay un nodo en ejecución para detener.")

    def run(self):
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()