#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas
import subprocess, os, signal
from open_manipulator_msgs.srv import (
    SetActuatorState, SetActuatorStateRequest, 
    SetJointPosition, SetJointPositionRequest, 
    SetKinematicsPose, SetKinematicsPoseRequest)
from open_manipulator_msgs.msg import JointPosition, KinematicsPose, OpenManipulatorState
import threading
from std_msgs.msg import Bool
class process:

    def __init__(self):
        super(process, self).__init__()
        rospy.init_node('pick_and_place', anonymous=True)
        self.namespace = rospy.get_param('~namespace', 'default_value') 
        self.present_kinematic_position = [0.0, 0.0, 0.0]  # Inicialización segura
        self.kinematics_pose = None
        global x_min, x_max, y_min, y_max,x,y
        global identificacion
        global autorizacion
        autorizacion=True
        x_min, x_max = 130, 203
        y_min, y_max = 95, 135
        x=None
        y=None
        identificacion = True
        self.kinematics_pose_subscriber = rospy.Subscriber(f'{self.namespace}/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.centroide_for_tracking = rospy.Subscriber(f"{self.namespace}/objeto1/Centroide", Point, self.objeto1_centroide)
        #self.objeto2 = rospy.Subscriber(f"{self.namespace}/objeto2/Centroide", Point, self.objeto2_centroide)
        self.publicador_agarrado = rospy.Publisher(f'{self.namespace}/state_agarre', Bool, queue_size=10)
        #Ejecución de movimiento del robot
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy(f'{self.namespace}/goal_task_space_path_position_only', SetKinematicsPose)
        self.set_tool_control_client = rospy.ServiceProxy(f'{self.namespace}/goal_tool_control', SetJointPosition)
        self.set_joint_position_client = rospy.ServiceProxy(f'{self.namespace}/goal_joint_space_path', SetJointPosition)
         # Lanzar hilo para Comparaciones
        global segundo_objeto
        segundo_objeto=None
        global running
        running = True
        self.comparison_thread = threading.Thread(target=self.ciclo_comparaciones)
        self.comparison_thread.start()

    def objeto2_centroide(self,msg):
        #print("esta tracking")
        # Extraer las coordenadas de CENTROIDE
        global x,y
        x = msg.x
        y = msg.y
        return x,y

    def joint_states_callback(self, msg):
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

    def setJointSpacePath(self, joint_name, joint_angle, path_time):
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

    def ciclo_comparaciones(self):
        rate = rospy.Rate(50)  # Ejecuta 50 veces por segundo
        global running
        while not rospy.is_shutdown() and running==True:
            self.Comparaciones()
            rate.sleep()  
        pos='pos1'
        self.movimiento_agarre(pos)
        rospy.sleep(3) 
        global segundo_objeto
        segundo_objeto=False
        while not rospy.is_shutdown() and segundo_objeto==True:
            self.Comparaciones()
            rate.sleep() 
        pos='pos2'
        self.movimiento_agarre(pos)

    def movimiento_agarre(self,pos):
        position = self.getPresentKinematicsPose()
        print(f" POSITION OBJETO RECONOCIDO{position}")
        kinematics_pose= [position[0], position[1], 0.05]
        path_time=3.0
        print(f"AGARRE: {kinematics_pose}")
        aka1=self.setTaskSpacePath(kinematics_pose,path_time)
        rospy.sleep(2)
        if aka1==True:
            position = self.getPresentKinematicsPose()
            print(f" POSITION OBJETO RECONOCIDO{position}")
            kinematics_pose= [position[0]+0.06, position[1], 0.05]
            path_time=3.0
            print(f"AGARRE: {kinematics_pose}")
            aka2=self.setTaskSpacePath(kinematics_pose,path_time)
            rospy.sleep(2)
        if aka2==True:
            joint_angle=[-0.025]
            gripper_closed1 = self.setToolControl(joint_angle)
            if gripper_closed1 is True:
                rospy.sleep(2)
                print(f"{gripper_closed1}")
                if pos== 'pos1':
                    rospy.sleep(1)
                    kinematics_pose= [0.13, -0.15, 0.07] #POSICION 1 ESTANTERIA
                    path_time=2.0
                if pos== 'pos2':
                    rospy.sleep(1)
                    kinematics_pose= [0.13, 0.15, 0.07] #POSICION 2 ESTANTERIA
                    path_time=2.0
                print(f"AGARRE: {kinematics_pose}")
                aka3=self.setTaskSpacePath(kinematics_pose,path_time)
                rospy.sleep(2)
                if aka3==True:
                    joint_angle=[-0.018]
                    gripper_closed2 = self.setToolControl(joint_angle)
                    rospy.sleep(2)
                    if gripper_closed2 is True:
                        joint_name = ["joint1", "joint2", "joint3", "joint4"]
                        joint_angle = [0.000, 0.000 , 0.000, 1.456]  # Ángulos de la posición inicial
                        path_time = 2.0  # Tiempo de movimiento
                        success = self.setJointSpacePath(joint_name, joint_angle, path_time)
                        rospy.sleep(2)
                        global autorizacion
                        autorizacion==False
                        self.objeto2 = rospy.Subscriber(f"{self.namespace}/objeto2/Centroide", Point, self.objeto2_centroide)
                        
                        
        #self.agarrar_segundo_objeto()

    #def agarrar_segundo_objeto()

                
    #def stop(self):
    #    global running
    #    running = False  # Detenemos el bucle de comparaciones
    #    self.comparison_thread.join()  # Esperamos a que el hilo termine

    def objeto1_centroide(self,msg):
        global autorizacion
        if autorizacion==True:
            #print("esta tracking")
            # Extraer las coordenadas de CENTROIDE
            global x,y
            x = msg.x
            y = msg.y
            return x,y
        else:
            pass
    
    def Comparaciones(self):
        if self.kinematics_pose is None:
            rospy.logwarn("AÚN NO HAY INFORMACIÓN DEL GRIPPER")
        if self.kinematics_pose is not None:
            global x_min, x_max, y_min, y_max,x,y, identificacion
            if x is not None and y is not None:
                if x < x_min: #X PEQUEÑO
                    position = self.getPresentKinematicsPose()
                    print(f" POSITION ACTUAL {position}")
                    kinematics_pose= [position[0], position[1]+0.01, position[2]]
                    path_time=2.0
                    print("PEQUEÑO MI X. esta a la izquierda del centroide, MOVER A LA DERECHA")
                    print(f"OBJETIVO: {kinematics_pose}")
                    print(f"{x,y}")
                    aka=self.setTaskSpacePath(kinematics_pose,path_time)
                    rospy.sleep(3)  
                    print(f"CENTROIDE:{x,y}")
                    if y < y_min: #Y ES GRANDE
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]+0.01, position[1], position[2]]
                        path_time=2.0
                        print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                        print(f"OBJETIVO: {kinematics_pose}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(3)
                        print(f"CENTROIDE:{x,y}")
                    if y > y_max: #Y ES PEQUEÑO
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]-0.01, position[1], position[2]]
                        path_time=2.0
                        print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                        print(f"OBJETIVO: {kinematics_pose}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(3)
                        print(f"CENTROIDE:{x,y}")
                
                if x > x_max: #X GRANDE
                    position = self.getPresentKinematicsPose()
                    print(f" POSITION ACTUAL {position}")
                    kinematics_pose= [position[0], position[1]-0.01, position[2]]
                    path_time=2.0
                    print("PEQUEÑO MI X. esta a la derecha del centroide, MOVER A LA IZQUIERDA")
                    print(f"OBJETIVO: {kinematics_pose}")
                    aka=self.setTaskSpacePath(kinematics_pose,path_time)
                    rospy.sleep(3)
                    print(f"CENTROIDE:{x,y}")
                    if y < y_min: #Y ES GRANDE
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]+0.01, position[1], position[2]]
                        path_time=2.0
                        print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(3)
                        print(f"CENTROIDE:{x,y}")
                    if y > y_max: #Y ES PEQUEÑO
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]-0.01, position[1], position[2]]
                        path_time=2.0
                        print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(3)
                        print(f"CENTROIDE:{x,y}")

                if x_min <= x <= x_max and y_min <= y <= y_max:
                    print("Centroide dentro del cuadro → Robot detenido")
                    global running,segundo_objeto
                    running = False
                    segundo_objeto==True
            else:
                print("AÚN NO HAY INFORMACIÓN")

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
        #print("entra a la funcion de cartesiana")
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

    def run(self):
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()
        
if __name__ == "__main__":
    try:
        nodo = process()
        nodo.run()
    except rospy.ROSInterruptException:
        pass