#!/usr/bin/env python
import rospy
import threading
import actionlib
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest)
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState, JointPosition

from action_carga_bienes_manipulador.msg import carga_bienesAction, carga_bienesFeedback, carga_bienesResult

class ServidorRecogida:
    def __init__(self):
        self.success = self.sucess1= self.success2= self.success3=self.success4=self.success5=self.success6=self.success7=self.success8=self.success9=False
        global x_min, x_max, y_min, y_max
        x_min, x_max = 130, 190
        y_min, y_max = 95, 135
        self.x=None
        self.y=None
        self.color_cx=None
        self.color_cy=None
        self.busca_por_forma=False
        self.busca_por_color=False
        self.present_kinematic_position = [0.0, 0.0, 0.0]
        self.kinematics_pose = None
        self.open_manipulator_is_moving = False
        self.open_manipulator_actuator_enabled = False
        self.existencia_objeto_forma= False
        self.existencia_objeto_color= False
        self.Listo_for_finalizar= False
        self.objeto_existe=False
        self.terminacion_tracking=False
        global running_revision
        global running 
        running=False #Running True es que deje de comparar
        running_revision=False
        rospy.init_node('servidor_carga')
        self.camera_callback = rospy.Subscriber("/gamora/usb_cam/image_raw", Image, self.camera_callback)
        self.kinematics_pose_subscriber = rospy.Subscriber('/gamora/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.joint_state_subscriber = rospy.Subscriber('/gamora/joint_states', JointState, self.joint_states_callback)

        self.set_actuator_state_client = rospy.ServiceProxy('/gamora/set_actuator_state', SetActuatorState)
        self.set_joint_position_client = rospy.ServiceProxy('/gamora/goal_joint_space_path', SetJointPosition)
        self.set_tool_control_client = rospy.ServiceProxy('/gamora/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy('/gamora/goal_task_space_path_position_only', SetKinematicsPose)

        if not self.setActuatorState(True):  # Llamada para habilitar los actuadores
            print("No se habilitaron los actuadores")
            return
        else:
            print("SE HABILITARON LOS ACTUADORES")
            rospy.wait_for_service('/gamora/goal_joint_space_path')
            joint_angle=[0.01]
            ggripper_closed = self.setToolControl(joint_angle) #AQUI AGARRA EL OBJETO
            joint_name = ["joint1", "joint2", "joint3", "joint4"]
            joint_angle = [0.0, -1.05, 0.35, 0.70]  # Ángulos de la posición inicial
            path_time = 2.0  # Tiempo de movimiento
            self.aka= self.setJointSpacePath(joint_name, joint_angle, path_time)
            self.server = actionlib.SimpleActionServer('carga_bienes', carga_bienesAction, self.execute_cb, False)
            # Inicia el servidor (lo pone a escuchar)
            self.server.start()
            rospy.loginfo("Servidor de recogida listo")
            

    def revision_centroide(self):
        rate = rospy.Rate(50)  # Ejecuta 50 veces por segundo
        #global running_revision
        while not rospy.is_shutdown() and running_revision==True:
            self.revisar_existencia_centroide()
            rate.sleep()  

    def revisar_existencia_centroide(self):
        if self.existencia_objeto_color==True:
            if self.color_cx is not None and self.color_cy is not None:
                #print("gentelo hay")                                   #REV 1 : YA SABEMOS QUE SI HAY CENTROIDE
                rospy.sleep(3)
                global running_revision
                if running_revision==True:
                    self.x = self.color_cx
                    self.y = self.color_cy
                    #print("CENTROIDE    EEEEEEEEEEEESTA GUARDANDO RESIDUOSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
                    return self.x, self.y
        if self.existencia_objeto_forma==True:
            if self.forma_cx is not None and self.forma_cy is not None:
                #print("gentelo hay")                                   #REV 1 : YA SABEMOS QUE SI HAY CENTROIDE
                rospy.sleep(3)
                if running_revision==True:
                    self.x = self.forma_cx
                    self.y = self.forma_cy
                    #print("CENTROIDE    EEEEEEEEEEEESTA GUARDANDO RESIDUOSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
                    return self.x, self.y


    def ciclo_comparaciones(self):
        rate = rospy.Rate(50)  # Ejecuta 50 veces por segundo
        global running
        while not rospy.is_shutdown() and running is not True:
            self.Comparaciones() 
            rate.sleep() 
        pass 
        

    def camera_callback(self, data):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return
        cv_image = cv2.resize(cv_image, (320, 240))
        if self.busca_por_forma==True or self.busca_por_color==True:
            
            #BUSCAR POR FORMA
            if self.busca_por_forma==True:
                if self.forma is not None:
                    if self.forma=='circulo':
                        self.lados=8
                    if self.forma=='cuadrado':
                        self.lados=4
                    if self.forma=='triangulo':
                        self.lados=3
                    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                    canny_min = 10
                    canny_max=150
                    canny=cv2.Canny(gray_image,canny_min,canny_max)
                    contours, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                        x, y, w, h = cv2.boundingRect(approx)
                        if len(approx) == self.lados:
                            M = cv2.moments(contour)
                            if M["m00"] != 0:
                                self.forma_cx = int(M["m10"] / M["m00"])
                                self.forma_cy = int(M["m01"] / M["m00"])
                                cv2.circle(cv_image, (self.forma_cx, self.forma_cy), 5, (0, 255, 0), -1)  # Dibujar centroide
                                cv2.drawContours(cv_image, [contour], -1, (0, 255, 255), 9)
                                #print("reconoció por forma")
                                self.existencia_objeto_forma=True

            #BUSCAR POR COLOR
            if self.busca_por_color==True:
                if self.color is not None:
                    if self.color=='verde':
                        hsv_min=np.array([40,50,50])
                        hsv_max=np.array([80,255,255])
                    if self.color=='azul':
                        hsv_min=np.array([100,50,50])
                        hsv_max=np.array([140,255,255])
                    if self.color=='rojo':
                        hsv_min=np.array([0,50,50])
                        hsv_max=np.array([10,255,255])

                    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                    mask_objeto1 = cv2.inRange(hsv_image, hsv_min, hsv_max)
                    contours, _ = cv2.findContours(mask_objeto1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if area > 100:  # filtra ruido pequeño
                            cv2.drawContours(cv_image, [contour], -1, (0, 0, 255), 2)  # rojo
                            M = cv2.moments(contour)
                            if M["m00"] != 0:
                                self.color_cx = int(M["m10"] / M["m00"])
                                self.color_cy = int(M["m01"] / M["m00"])
                                cv2.circle(cv_image, (self.color_cx, self.color_cy), 5, (0, 0, 255), -1)
                                #print("reconoció por color")
                                self.existencia_objeto_color=True


        cv2.imshow('Imagen con reconocimiento', cv_image)
        cv2.waitKey(1)

    def execute_cb(self, goal):
        """
        Esta función se llama automáticamente cuando un cliente envía un goal.
        El 'goal' es un objeto que contiene: color y forma
        """
        global x_min, x_max, y_min, y_max
        x_min, x_max = 130, 190
        y_min, y_max = 95, 135
        self.busca_por_forma=False
        self.busca_por_color=False
        self.color_cx=None
        self.color_cy=None
        self.x=None
        self.y=None
        self.present_kinematic_position = [0.0, 0.0, 0.0]
        self.kinematics_pose = None
        self.open_manipulator_is_moving = False
        self.open_manipulator_actuator_enabled = False
        self.existencia_objeto_forma= False
        self.existencia_objeto_color= False
        self.Listo_for_finalizar= False
        self.objeto_existe=False
        self.terminacion_tracking=False
        feedback = carga_bienesFeedback()
        self.result = carga_bienesResult()

        rospy.loginfo("Petición recibida: Color: %s, Forma: %s", goal.color, goal.forma)
        self.color, self.forma = goal.color, goal.forma

        sesion_1=[-0.650, -1.05, 0.35, 0.70] #Cuabre los espacios E1,E2,E5, E6
        sesion_2=[-1.900, -1.05, 0.35, 0.70] #Observa los espacios E3, E4, E7, E8
        sesion_3=[-1.900, 0.447, 0.583, -0.930]  #Observa los espacios E9,E10, E13, E14
        sesion_4=[-0.713, 0.447, 0.583, -0.930] #Observa los espacios E11, E12, E15, E16

        pos_reco_forma=[0.00,0.00,0.00,1.456]
        
#COLOR -----> ESTANTERÍA
#FORMA ------> PISO POS RECO

        if self.color or self.forma:
            feedback.estado_actual = "Desocupado"
            self.server.publish_feedback(feedback)
            print (f"{self.color, self.forma}")
            self.agarre_color=False
            self.agarre_forma=False
            self.tracking_forma=False
            self.tracking_color=False
            self.busca_por_forma=False
            self.busca_por_color=False
            self.color_cx=None
            self.color_cy=None
            self.x=None
            self.y=None
            global running
            running = False
            global running_revision 
            running_revision= True
            self.hilo_comparacion = threading.Thread(target=self.ciclo_comparaciones)
            self.hilo_revision_centroide = threading.Thread(target=self.revision_centroide)
            self.hilo_revision_centroide.start()
            self.hilo_comparacion.start()
            
            feedback.estado_actual = f"Petición de carga recibida .Forma: {self.forma}, Color: {self.color}"
            self.server.publish_feedback(feedback)
            print(f"{feedback.estado_actual}")

            if self.color=='azul' or self.color=='rojo' or self.color=='verde' or self.forma =='circulo' or self.forma=='cuadrado' or self.forma=='triangulo' :
################################################################################################################################################################################################################################                
                                                                           #COLOR Y FORMA
                if (self.forma =='circulo' or self.forma=='cuadrado' or self.forma=='triangulo') and (self.color=='azul' or self.color=='rojo' or self.color=='verde'):
                    feedback.estado_actual = "Cargando"
                    self.server.publish_feedback(feedback)
                    rospy.sleep(1)
                    self.tracking_color= True
                    self.agarre_color=True
                    print("COLOOOOOOOOOY FORMAAAAAAAAAAA")
                    #busca tanto color como forma
                    self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                    self.busca_por_color=False
                    #Se dirige a la posicion S1 de la estantería
                    joint_angle = 0.01
                    joint_name = ["joint1", "joint2", "joint3", "joint4"]
                    joint_angle = sesion_1  # Ángulos de la posición inicial
                    path_time = 2.0  # Tiempo de movimiento
                    self.success = self.setJointSpacePath(joint_name, joint_angle, path_time)
                    rospy.sleep(2)
                    if self.success==True:
                        self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                        self.busca_por_color=True
                        rospy.sleep(2)   #como el callback camera se reproduce cada segundo, entonces que espere 2
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)


                            #INICIA BUSQUQEDA POR FORMA
                            print("FORMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")  
                            self.tracking_forma=True
                            self.agarre_forma=True
                            self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                            self.busca_por_color=False
                            #busca solo color
                            #Se dirige a la posicion S1 de la estantería
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = pos_reco_forma  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success4 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                            rospy.sleep(2)
                            if self.success4==True:
                                self.busca_por_forma= True #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                                self.busca_por_color=False
                                rospy.sleep(2)   #como el callback camera se reproduce cada segundo, entonces que espere 2
                                if self.existencia_objeto_forma == True:
                                    while not self.terminacion_tracking:
                                        rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                                    rospy.sleep(1)
                                    self.movimiento_agarre()
                                    while not self.terminacion_agarre:
                                        rospy.sleep(0.1)
                                    self.termin_agarre()
                        
                            
                        else:
                            #Si no hay objeto en S2 entonces mirar S2
                            self.busca_por_color=False
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = sesion_2  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success1 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                            #rospy.sleep(2)
                    if self.success1==True:
                        self.busca_por_color=True
                        rospy.sleep(2)
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)
                            #INICIA BUSQUQEDA POR FORMA
                            print("FORMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")  
                            self.tracking_color=False
                            self.agarre_color=False
                            self.tracking_forma=True
                            self.agarre_forma=True
                            self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                            self.busca_por_color=False
                            #busca solo color
                            #Se dirige a la posicion S1 de la estantería
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = pos_reco_forma  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success4 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                            rospy.sleep(2)
                            if self.success4==True:
                                self.busca_por_forma= True #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                                self.busca_por_color=False
                                rospy.sleep(2)   #como el callback camera se reproduce cada segundo, entonces que espere 2
                                if self.existencia_objeto_forma == True:
                                    while not self.terminacion_tracking:
                                        rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                                    rospy.sleep(1)
                                    self.movimiento_agarre()
                                    while not self.terminacion_agarre:
                                        rospy.sleep(0.1)
                                    self.termin_agarre()
                        

                        else:
                            #Si no hay objeto en S3 entonces mirar S2
                            self.busca_por_color=False
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = sesion_3  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success2 = self.setJointSpacePath(joint_name, joint_angle, path_time)

                    if self.success2==True:                        
                        self.busca_por_color=True
                        rospy.sleep(2)
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)
                            #INICIA BUSQUQEDA POR FORMA
                            print("FORMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")  
                            self.tracking_color=False
                            self.agarre_color=False
                            self.tracking_forma=True
                            self.agarre_forma=True
                            self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                            self.busca_por_color=False
                            #busca solo color
                            #Se dirige a la posicion S1 de la estantería
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = pos_reco_forma  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success4 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                            rospy.sleep(2)
                            if self.success4==True:
                                self.busca_por_forma= True #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                                self.busca_por_color=False
                                rospy.sleep(2)   #como el callback camera se reproduce cada segundo, entonces que espere 2
                                if self.existencia_objeto_forma == True:
                                    while not self.terminacion_tracking:
                                        rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                                    rospy.sleep(1)
                                    self.movimiento_agarre()
                                    while not self.terminacion_agarre:
                                        rospy.sleep(0.1)
                                    self.termin_agarre()
                            
                        else:
                            #Si no hay objeto en S1 entonces mirar S2
                            self.busca_por_color=False
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = sesion_4  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success3 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                                                    

                    if self.success3==True:
                        self.busca_por_color=True
                        rospy.sleep(2)  
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            #INICIA BUSQUQEDA POR FORMA
                            print("FORMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")  
                            self.tracking_color=False
                            self.agarre_color=False
                            self.tracking_forma=True
                            self.agarre_forma=True
                            self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                            self.busca_por_color=False
                            #busca solo color
                            #Se dirige a la posicion S1 de la estantería
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = pos_reco_forma  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success4 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                            rospy.sleep(1)
                            if self.success4==True:
                                self.busca_por_forma= True #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                                self.busca_por_color=False
                                rospy.sleep(2)   #como el callback camera se reproduce cada segundo, entonces que espere 2
                                if self.existencia_objeto_forma == True:
                                    print("sensó el objeto")
                                    while not self.terminacion_tracking:
                                        print("entró al tracking")
                                        rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                                    rospy.sleep(1)
                                    self.movimiento_agarre()
                                    while not self.terminacion_agarre:
                                        rospy.sleep(0.1)
                                    self.termin_agarre()
                                    
                                else:
                                    if self.existencia_objeto_forma == False:
                                        self.tracking_forma=False    
                                        self.agarre_forma=False                   
                                        rospy.sleep(2)
                                        self.busca_por_forma=False
                                        self.result.estado_final = f"Error: Objeto forma {self.forma} no disponible"
                                        self.server.set_aborted(self.result)
                            
                        else:
                            if self.existencia_objeto_color == False:                       
                                rospy.sleep(2)
                                self.busca_por_color=False
                                self.result.estado_final = f"Error: Objeto color {self.color} no disponible"
                                self.server.set_aborted(self.result)



################################################################################################################################################################################################
                                                                                # FORMA
                if (self.forma =='circulo' or self.forma=='cuadrado' or self.forma=='triangulo') and (self.color!='azul' or self.color!='rojo' or self.color!='verde'): 
                    feedback.estado_actual = "Cargando"
                    self.server.publish_feedback(feedback)
                    rospy.sleep(1)
                    print("FORMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")   
                    self.tracking_forma=True
                    self.agarre_forma=True
                    self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                    self.busca_por_color=False
                    #busca solo color
                    #Se dirige a la posicion S1 de la estantería
                    joint_angle = 0.01
                    joint_name = ["joint1", "joint2", "joint3", "joint4"]
                    joint_angle = pos_reco_forma  # Ángulos de la posición inicial
                    path_time = 2.0  # Tiempo de movimiento
                    self.success5 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                    rospy.sleep(2)
                    if self.success5==True:
                        self.busca_por_forma= True #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                        self.busca_por_color=False
                        rospy.sleep(2)   #como el callback camera se reproduce cada segundo, entonces que espere 2
                        if self.existencia_objeto_forma == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)
                            self.termin_agarre()
                            
                        else:
                            if self.existencia_objeto_forma == False:
                                self.tracking_forma=False    
                                self.agarre_forma=False                   
                                rospy.sleep(2)
                                self.busca_por_forma=False
                                self.result.estado_final = f"Error: Objeto forma {self.forma} no disponible"
                                self.server.set_aborted(self.result)
                            

####################################################################################################################################################################################3
                                                                                # COLOR
                if (self.color=='azul' or self.color=='rojo' or self.color=='verde') and (self.forma !='circulo' or self.forma!='cuadrado' or self.forma!='triangulo'):
                    feedback.estado_actual = "Cargando"
                    self.server.publish_feedback(feedback)
                    rospy.sleep(1)
                    self.tracking_color= True
                    self.agarre_color=True
                    print("COOOOOOLLLLLOOOOOOOOOOOOOR")
                    self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                    self.busca_por_color=False
                    #busca solo color
                    #Se dirige a la posicion S1 de la estantería
                    joint_angle = 0.01
                    joint_name = ["joint1", "joint2", "joint3", "joint4"]
                    joint_angle = sesion_1  # Ángulos de la posición inicial
                    path_time = 2.0  # Tiempo de movimiento
                    self.success6 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                    rospy.sleep(2)
                    if self.success6==True:
                        self.busca_por_forma= False #ESTO ME ACTIVA EL RECO EN CALLBACK CAMERA
                        self.busca_por_color=True
                        rospy.sleep(2)   #como el callback camera se reproduce cada segundo, entonces que espere 2
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)
                            self.termin_agarre()
                            
                        else:
                            #Si no hay objeto en S2 entonces mirar S2
                            self.busca_por_color=False
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = sesion_2  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success7 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                            #rospy.sleep(2)
                    if self.success7==True:
                        self.busca_por_color=True
                        rospy.sleep(2)
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)
                            self.termin_agarre()

                        else:
                            #Si no hay objeto en S3 entonces mirar S2
                            self.busca_por_color=False
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = sesion_3  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success8 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                
                    
                    if self.success8==True:                        
                        self.busca_por_color=True
                        rospy.sleep(2)
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)
                            self.termin_agarre()
                        else:
                            #Si no hay objeto en S1 entonces mirar S2
                            self.busca_por_color=False
                            joint_angle = 0.01
                            joint_name = ["joint1", "joint2", "joint3", "joint4"]
                            joint_angle = sesion_4  # Ángulos de la posición inicial
                            path_time = 2.0  # Tiempo de movimiento
                            self.success9 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                                                    

                    if self.success9==True:
                        self.busca_por_color=True
                        rospy.sleep(2)  
                        if self.existencia_objeto_color == True:
                            while not self.terminacion_tracking:
                                rospy.sleep(0.1)  # Espera corta para no bloquear el sistema
                            rospy.sleep(1)
                            self.movimiento_agarre()
                            while not self.terminacion_agarre:
                                rospy.sleep(0.1)
                            self.termin_agarre()
                        else:
                        
                            if self.existencia_objeto_color == False:                       
                                rospy.sleep(2)
                                self.busca_por_color=False
                                self.agarre_color=False
                                self.tracking_color=False
                                self.result.estado_final = f"Error: Objeto color {self.color} no disponible"
                                self.server.set_aborted(self.result)
     
            else:
                self.result.estado_final = "Error: Parámetros inválidos"
                self.server.set_aborted(self.result)
#######################################################################################################################3


            #LA CONDICION ES QUE SI YA TERMINÓ DE ESCANEAR ENTONCES QUE DE FINALIZADO




        else:
            print (f" paila no llegó el dato vacío{self.color}")
            self.result.estado_final = "Error: No hay parámetros"
            self.server.set_aborted(self.result)
            
    def run():
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()

###############################################################################    CALLBACKS Y SERVICIOS DE MOVIMIEINTO  #########################################################################
    def movimiento_agarre(self):
        if self.agarre_color==True:
            position = self.getPresentKinematicsPose()
            print(f" POSITION OBJETO RECONOCIDO{position}")
            articuler=self.getPresentJointAngle()
            joint_angle = 0.01
            joint_name = ["joint1", "joint2", "joint3", "joint4"]   
            ########NOTA MENTAL: SI DEPRONTO EMPUJA EL OBJETO MEJOR ES ENVIARLO A MOVIMIENTO LIMPIO, GUARDANDO ESTA POS KINEMATICS, SE ENVIA A UNA POS INICIAL ARRIBA Y SE ENVIA LIMPIAMENTE CON KINEMATICS
            joint_angle = [articuler[0], articuler[1]+0.318, articuler[2]-0.651, articuler[3]+0.333]  # [0, 0.318, -0.651, 0.333]
            path_time = 2.0  # Tiempo de movimiento
            aka1 = self.setJointSpacePath(joint_name, joint_angle, path_time)
            print(f"AGARRE: {articuler}")
            rospy.sleep(2)
            if aka1==True:
                articuler=self.getPresentJointAngle()
                joint_angle = 0.01
                joint_name = ["joint1", "joint2", "joint3", "joint4"]   
                joint_angle = [articuler[0]+0.014, articuler[1]+0.264, articuler[2]-0.227, articuler[3]-0.037]   #[0.014, 0.264, -0.227, -0.037]
                path_time = 2.0  # Tiempo de movimiento
                aka2 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                rospy.sleep(1)
            if aka2==True:
                joint_angle=[-0.01]
                gripper_closed = self.setToolControl(joint_angle) #AQUI AGARRA EL OBJETO
                print(f"{gripper_closed}")    #AHORA SACA EL OBJETO DE LA ESTANTERIA Y LO LLEVA A LA POSICION DE LA BASE DEL ROBOT
                rospy.sleep(2)
            if gripper_closed is True:
                joint_angle = 0.01
                joint_name = ["joint1", "joint2", "joint3", "joint4"]   
                joint_angle = [1.514, 0.732, -0.630, 0.9]  #POSICION DE LA BASE DEL ROBOT #[1.514, 0.732, -0.630, 0.9]
                path_time = 2.0  # Tiempo de movimiento
                aka3 = self.setJointSpacePath(joint_name, joint_angle, path_time)    
                rospy.sleep(2) 
            if aka3==True:
                joint_angle=[0.01]
                gripper_open = self.setToolControl(joint_angle)
                rospy.sleep(1)
                print(f"{gripper_open}")
                self.existencia_objeto_color = False
                self.existencia_objeto_forma = False
                self.busca_por_color=False
                self.busca_por_forma=False
                self.terminacion_agarre= True
            if gripper_open==True:
                rospy.sleep(1)
                return self.terminacion_agarre
            
        if self.agarre_forma==True:
            position = self.getPresentKinematicsPose()
            print(f" POSITION OBJETO RECONOCIDO{position}")
            articuler=self.getPresentJointAngle()
            joint_angle = 0.01
            joint_name = ["joint1", "joint2", "joint3", "joint4"]   
            ########NOTA MENTAL: SI DEPRONTO EMPUJA EL OBJETO MEJOR ES ENVIARLO A MOVIMIENTO LIMPIO, GUARDANDO ESTA POS KINEMATICS, SE ENVIA A UNA POS INICIAL ARRIBA Y SE ENVIA LIMPIAMENTE CON KINEMATICS
            joint_angle = [articuler[0], articuler[1]+0.318, articuler[2]-0.651, articuler[3]+0.333]  # [0, 0.318, -0.651, 0.333]
            path_time = 2.0  # Tiempo de movimiento
            aka1 = self.setJointSpacePath(joint_name, joint_angle, path_time)
            print(f"AGARRE: {articuler}")
            rospy.sleep(2)
            if aka1==True:
                articuler=self.getPresentJointAngle()
                joint_angle = 0.01
                joint_name = ["joint1", "joint2", "joint3", "joint4"]   
                joint_angle = [articuler[0]+0.014, articuler[1]+0.264, articuler[2]-0.227, articuler[3]-0.037]   #[0.014, 0.264, -0.227, -0.037]
                path_time = 2.0  # Tiempo de movimiento
                aka2 = self.setJointSpacePath(joint_name, joint_angle, path_time)
                rospy.sleep(1)
            if aka2==True:
                joint_angle=[-0.01]
                gripper_closed = self.setToolControl(joint_angle) #AQUI AGARRA EL OBJETO
                print(f"{gripper_closed}")    #AHORA SACA EL OBJETO DE LA ESTANTERIA Y LO LLEVA A LA POSICION DE LA BASE DEL ROBOT
                rospy.sleep(2)
            if gripper_closed is True:
                joint_angle = 0.01
                joint_name = ["joint1", "joint2", "joint3", "joint4"]   
                joint_angle = [1.514, 0.732, -0.630, 0.9]  #POSICION DE LA BASE DEL ROBOT #[1.514, 0.732, -0.630, 0.9]
                path_time = 2.0  # Tiempo de movimiento
                aka3 = self.setJointSpacePath(joint_name, joint_angle, path_time)    
                rospy.sleep(2) 
            if aka3==True:
                joint_angle=[0.01]
                gripper_open = self.setToolControl(joint_angle)
                rospy.sleep(1)
                print(f"{gripper_open}")
                self.existencia_objeto_color = False
                self.existencia_objeto_forma = False
                self.busca_por_color=False
                self.busca_por_forma=False
                self.terminacion_agarre= True
            if gripper_open==True:
                rospy.sleep(1)
                return self.terminacion_agarre

    def termin_agarre(self):
        self.color_cx=None
        self.color_cy=None
        self.busca_por_forma=False
        self.busca_por_color=False
        self.present_kinematic_position = [0.0, 0.0, 0.0]
        self.kinematics_pose = None
        self.open_manipulator_is_moving = False
        self.open_manipulator_actuator_enabled = False
        self.existencia_objeto_forma= False
        self.existencia_objeto_color= False
        self.Listo_for_finalizar= False
        self.objeto_existe=False
        self.running_revision=False
        self.running=True
        self.terminacion_tracking=False
        self.terminacion_agarre=True
        self.color=None
        self.forma=None
        print(f"{self.y, self.x, self.color_cx, self.color_cy, self.color, self.forma}")
        self.result.estado_final = "Finalizado"
        self.server.set_succeeded(self.result)
            



    def Comparaciones(self):
        global running
        global running_revision
        if self.kinematics_pose is None:
            #print("no hay objeto")
            pass
        if self.kinematics_pose is not None:
            #print(f"{self.x, self.y}")
            x_min, x_max = 130, 200
            y_min, y_max = 95, 140
            if self.x is not None and self.y is not None and self.x >0 and self.y>0: 
                if self.tracking_color== True:
                    #print("COMPARACIONES_ EEEEEEEEEEEESTA GUARDANDO RESIDUOSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
                    #print(f"{self.x,self.y}")
                    if self.x < x_min: #X PEQUEÑO
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]+0.01, position[1], position[2]]
                        path_time=2.0
                        print("PEQUEÑO MI X. esta a la izquierda del centroide, MOVER A LA DERECHA")
                        print(f"OBJETIVO: {kinematics_pose}")
                        print(f"{self.x,self.y}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time)
                        rospy.sleep(3)  
                        print(f"CENTROIDE:{self.x,self.y}")
                        if self.y < y_min: #Y ES GRANDE
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]+0.01]
                            path_time=2.0
                            print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                            print(f"OBJETIVO: {kinematics_pose}")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")
                        if self.y > y_max: #Y ES PEQUEÑO
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]-0.01]
                            path_time=2.0
                            print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                            print(f"OBJETIVO: {kinematics_pose}")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")
                    
                    if self.x > x_max: #X GRANDE
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]-0.01, position[1], position[2]]
                        path_time=2.0
                        print("PEQUEÑO MI X. esta a la derecha del centroide, MOVER A LA IZQUIERDA")
                        print(f"OBJETIVO: {kinematics_pose}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time)
                        rospy.sleep(3)
                        print(f"CENTROIDE:{self.x,self.y}")
                        if self.y < y_min: #Y ES GRANDE
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]+0.01]
                            path_time=2.0
                            print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(1)
                            print(f"CENTROIDE:{self.x,self.y}")
                        if self.y > y_max: #Y ES PEQUEÑO
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]-0.01]
                            path_time=2.0
                            print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")

                    if self.x < x_max and self.x>x_min: #X ES IDEAL PERO Y NO
                        if self.y < y_min: #Y ES GRANDE
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]+0.01]
                            path_time=2.0
                            print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(1)
                            print(f"CENTROIDE:{self.x,self.y}")
                        if self.y > y_max: #Y ES PEQUEÑO
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]-0.01]
                            path_time=2.0
                            print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")


                    if self.x>=x_min and self.x<=x_max and self.x>=y_min and self.y<=y_max:
                        print("Centroide dentro del cuadro → Robot detenido")
                        self.terminacion_tracking=True
                        global running 
                        #global running_revision
                        running = True
                        running_revision=False
                        self.color_cx=None
                        self.color_cy=None
                        self.y=None
                        self.x=None
                        self.tracking_color= False
                        self.tracking_forma=False

                if self.tracking_forma==True:        
                    if self.x < x_min: #X PEQUEÑO
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0], position[1]+0.01, position[2]]
                        path_time=2.0
                        print("PEQUEÑO MI X. esta a la izquierda del centroide, MOVER A LA DERECHA")
                        print(f"OBJETIVO: {kinematics_pose}")
                        print(f"{self.x,self.y}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time)
                        rospy.sleep(3)  
                        print(f"CENTROIDE:{self.x,self.y}")
                        if self.y < y_min: #Y ES GRANDE
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0]+0.01, position[1], position[2]]
                            path_time=2.0
                            print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                            print(f"OBJETIVO: {kinematics_pose}")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")
                        if self.y > y_max: #Y ES PEQUEÑO
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0]-0.01, position[1], position[2]]
                            path_time=2.0
                            print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                            print(f"OBJETIVO: {kinematics_pose}")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")
                    
                    if self.x > x_max: #X GRANDE
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0], position[1]-0.01, position[2]]
                        path_time=2.0
                        print("PEQUEÑO MI X. esta a la derecha del centroide, MOVER A LA IZQUIERDA")
                        print(f"OBJETIVO: {kinematics_pose}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time)
                        rospy.sleep(3)
                        print(f"CENTROIDE:{self.x,self.y}")
                        if self.y < y_min: #Y ES GRANDE
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0]+0.01, position[1], position[2]]
                            path_time=2.0
                            print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")
                        if self.y > y_max: #Y ES PEQUEÑO
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0]-0.01, position[1], position[2]]
                            path_time=2.0
                            print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")

                    if self.x < x_max and self.x>x_min: #X ES IDEAL PERO Y NO
                        if self.y < y_min: #Y ES GRANDE
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]+0.01]
                            path_time=2.0
                            print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(1)
                            print(f"CENTROIDE:{self.x,self.y}")
                        if self.y > y_max: #Y ES PEQUEÑO
                            position = self.getPresentKinematicsPose()
                            print(f" POSITION ACTUAL {position}")
                            kinematics_pose= [position[0], position[1], position[2]-0.01]
                            path_time=2.0
                            print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                            aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                            rospy.sleep(3)
                            print(f"CENTROIDE:{self.x,self.y}")


                    if self.x>=x_min and self.x<=x_max and self.x>=y_min and self.y<=y_max:
                        print("Centroide dentro del cuadro → Robot detenido")
                        self.terminacion_tracking=True                             
                        #global running_revision
                        running = True
                        running_revision=False
                        self.color_cx=None
                        self.color_cy=None
                        self.y=None
                        self.x=None
                        self.tracking_color= False
                        self.tracking_forma=False
            else:
                self.objeto_existe=False
                #print("AÚN NO HAY INFORMACIÓN")

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

            #print(f" si le llegan las coordenadas{kinematics_pose}")
            # Mantener la orientación actual del efector final
            srv.kinematics_pose.pose.orientation = self.kinematics_pose.orientation

            srv.path_time = path_time
            response = self.goal_task_space_path_position_only_client(srv)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
        
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
#_________________________________________________________________________________________________________________________________________________________________________________________________
if __name__ == '__main__':
    node =ServidorRecogida()
    ServidorRecogida.run() 
