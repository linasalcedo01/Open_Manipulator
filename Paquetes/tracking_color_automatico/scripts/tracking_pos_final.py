#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point  # Tipo de mensaje para coordenadas
import subprocess, os, signal
from open_manipulator_msgs.srv import ( SetKinematicsPose, SetKinematicsPoseRequest, SetJointPosition, SetJointPositionRequest)
from open_manipulator_msgs.msg import KinematicsPose
import threading
from std_msgs.msg import Bool
class process:

    def __init__(self):
        super(process, self).__init__()
        rospy.init_node('tracking', anonymous=True)
        self.namespace = rospy.get_param('~namespace', 'default_value') 
        self.present_kinematic_position = [0.0, 0.0, 0.0]  # Inicialización segura
        self.kinematics_pose = None
        global x_min, x_max, y_min, y_max,x,y
        global identificacion
        x_min, x_max = 130, 190
        y_min, y_max = 95, 135
        x=None
        y=None
        identificacion = False
        self.kinematics_pose_subscriber = rospy.Subscriber(f'{self.namespace}/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.centroide_for_tracking = rospy.Subscriber(f"{self.namespace}/objeto/Centroide", Point, self.centroide)
        self.publicador_agarrado = rospy.Publisher(f'{self.namespace}/state_agarre', Bool, queue_size=10)
        #Ejecución de movimiento del robot
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy(f'{self.namespace}/goal_task_space_path_position_only', SetKinematicsPose)
        self.set_tool_control_client = rospy.ServiceProxy(f'{self.namespace}/goal_tool_control', SetJointPosition)
         # Lanzar hilo para Comparaciones
        global running
        running = True
        self.comparison_thread = threading.Thread(target=self.ciclo_comparaciones)
        self.comparison_thread.start()

    def ciclo_comparaciones(self):
        rate = rospy.Rate(50)  # Ejecuta 50 veces por segundo
        while not rospy.is_shutdown() and running==True:
            self.Comparaciones()
            rate.sleep()  
        self.movimiento_agarre()

    def movimiento_agarre(self):
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
            kinematics_pose= [position[0]+0.05, position[1], 0.05]
            path_time=3.0
            print(f"AGARRE: {kinematics_pose}")
            aka2=self.setTaskSpacePath(kinematics_pose,path_time)
            rospy.sleep(2)
        if aka2==True:
            joint_angle=[-0.01]
            gripper_closed = self.setToolControl(joint_angle) #AQUI AGARRA EL OBJETO
            if gripper_closed is True:
                rospy.sleep(2)
                print(f"{gripper_closed}")
                kinematics_pose= [0.13, -0.15, 0.07]
                path_time=2.0
                print(f"AGARRE: {kinematics_pose}")
                aka3=self.setTaskSpacePath(kinematics_pose,path_time)
                rospy.sleep(2)
                if aka3==True:
                    joint_angle=[0.01]
                    gripper_closed = self.setToolControl(joint_angle)
                    rospy.sleep(2)
                
    def stop(self):
        global running
        running = False  # Detenemos el bucle de comparaciones
        self.comparison_thread.join()  # Esperamos a que el hilo termine

    def centroide(self,msg):
        #print("esta tracking")
        # Extraer las coordenadas de CENTROIDE
        global x,y
        x = msg.x
        y = msg.y
        return x,y
    
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
                    global running
                    running = False
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