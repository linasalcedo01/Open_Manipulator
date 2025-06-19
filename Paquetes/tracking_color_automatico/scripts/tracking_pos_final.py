#!/usr/bin/env python3

#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2  # OpenCV
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String
import time
import tty
import termios
import threading
import sys
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
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
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


        self.publicador_agarrado = rospy.Publisher(f'{self.namespace}/state_agarre', Bool, queue_size=10)
        self.kinematics_pose_subscriber = rospy.Subscriber(f'{self.namespace}/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.centroide_for_tracking = rospy.Subscriber(f"{self.namespace}/objeto/Centroide", Point, self.centroide)
        self.joint_state_subscriber = rospy.Subscriber(f'{self.namespace}/joint_states', JointState, self.joint_states_callback)
        self.kinematics_pose_subscriber = rospy.Subscriber(f'{self.namespace}/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.publicador_dimensiones= rospy.Publisher(f'{self.namespace}/dimensiones_imagen', Point, queue_size=10)
        self.sub_contacts = rospy.Subscriber (f'{self.namespace}/contact_states', ContactsState, self.get_contacts)
        #Ejecución de movimiento del robot

        self.set_joint_position_client = rospy.ServiceProxy(f'{self.namespace}/goal_joint_space_path', SetJointPosition)
        self.set_tool_control_client = rospy.ServiceProxy(f'{self.namespace}/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy(f'{self.namespace}/goal_task_space_path_position_only', SetKinematicsPose)

  
        self.detectedObject = None
        self.detach_srv = rospy.ServiceProxy(f'{self.namespace}/link_attacher_node/detach',Attach)
        self.attach_srv = rospy.ServiceProxy(f'{self.namespace}/link_attacher_node/attach',Attach)
         # Lanzar hilo para Comparaciones
        global running
        running = True
        self.suspension=False
        self.comparison_thread = threading.Thread(target=self.ciclo_comparaciones)
        self.comparison_thread.start()

    def ciclo_comparaciones(self):
        rate = rospy.Rate(50)  # Ejecuta 50 veces por segundo
        while not rospy.is_shutdown() and running==True:
            self.Comparaciones()
            rate.sleep()  
        print("Arrancando movimiento............")
        self.movimiento_agarre()

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

    def movimiento_agarre(self):
        rospy.sleep(1)
        self.posicion_recogida()
        rospy.sleep(0.5)
        self.cerrar_gripper()
        rospy.sleep(0.5)
        self.posicion_preparando_destino()
        rospy.sleep(1)
        self.posicion_destino()
        rospy.sleep(2)
        self.abrir_gripper()
        rospy.sleep(1)
        self.posicion_inicial()
        print("-------TAREA TERMINADA ---------")

    def posicion_preparando_destino(self):
        print("PREPARANDO DESTINO")
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[-1.56, -1.190, 1.190, 0.240]
        path_time = 0.5  # Tiempo de movimiento
        success1 = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def posicion_recogida(self):
        print("LLEVANDO A AGARRAR OBJETO")
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        print("1")
        position = self.getPresentKinematicsPose()
        kinematics_pose= [position[0]+0.01, position[1], position[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose,path_time)
        rospy.sleep(0.1)
        position1 = self.getPresentKinematicsPose()
        kinematics_pose1= [position1[0]+0.01, position1[1], position1[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose1,path_time)
        rospy.sleep(0.1)
        position2 = self.getPresentKinematicsPose()
        kinematics_pose2= [position2[0]+0.01, position2[1], position2[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose2,path_time)
        rospy.sleep(0.1)
        position3 = self.getPresentKinematicsPose()
        kinematics_pose3= [position3[0]+0.01, position3[1], position3[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose3,path_time)
        rospy.sleep(0.1)
        position4 = self.getPresentKinematicsPose()
        kinematics_pose4= [position4[0]+0.01, position4[1], position4[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose4,path_time)
        position12 = self.getPresentKinematicsPose()
        rospy.sleep(0.1)
        kinematics_pose12= [position12[0]+0.01, position12[1], position12[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose12,path_time)
        rospy.sleep(0.1)
        position13 = self.getPresentKinematicsPose()
        rospy.sleep(0.1)
        kinematics_pose13= [position13[0]+0.01, position13[1], position13[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose13,path_time)
        position4 = self.getPresentKinematicsPose()
        rospy.sleep(0.1)
        kinematics_pose4= [position4[0]+0.01, position4[1], position4[2]]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose4,path_time)
        print("2")
        position5 = self.getPresentKinematicsPose()
        rospy.sleep(0.1)
        kinematics_pose5= [position5[0], position5[1], position5[2]-0.01]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose5,path_time)
        rospy.sleep(0.1)
        position6 = self.getPresentKinematicsPose()
        kinematics_pose6= [position6[0], position6[1], position6[2]-0.01]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose6,path_time)
        rospy.sleep(0.1)
        position7 = self.getPresentKinematicsPose()
        kinematics_pose7= [position7[0], position7[1], position7[2]-0.01]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose7,path_time)
        rospy.sleep(0.1)
        position8 = self.getPresentKinematicsPose()
        kinematics_pose8= [position8[0], position8[1], position8[2]-0.01]
        path_time=0.8
        aka=self.setTaskSpacePath(kinematics_pose8,path_time)
        rospy.sleep(0.1)


                
    def posicion_destino(self):
        print("LLEVANDO A DESTINO")
        print("Llevando a posicion destino de la estantería")
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[-1.73, 0.24, 0.47, -0.60]
        path_time = 0.5  # Tiempo de movimiento
        success1 = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def posicion_inicial(self):
        print("LLEVANDO A DESTINO")
        print("Llevando a posicion destino de la estantería")
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[0.00, 0.00, 0.00, 0.00]
        path_time = 0.5  # Tiempo de movimiento
        success1 = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def abrir_gripper(self):
        if self.namespace=="":
            print("SOLTANDO OBJETO")
            joint_angle=[0.01]
            soul=self.setToolControl(joint_angle)
            if soul==True:
                rospy.sleep(0.2)
                self.Tool_attach(joint_angle) #puede haber problema porque se envia justo despues d eordenar mover el gripper
        if (self.namespace=="/nebula" or self.namespace=="/gamora"):
            print("SOLTANDO OBJETO")
            joint_angle=[-0.018]
            gripper_closed = self.setToolControl(joint_angle)

    def cerrar_gripper(self):
        if self.namespace=="":
            print("AGARRANDO OBJETO")
            joint_angle1=[-0.01]
            soul=self.setToolControl(joint_angle1)
            if soul==True:
                rospy.sleep(0.2)
                self.Tool_attach(joint_angle1) #puede haber problema porque se envia justo despues d eordenar mover el gripper
        if (self.namespace=="/nebula" or self.namespace=="/gamora"):
            print("AGARRANDO OBJETO")
            joint_angle=[-0.025]
            gripper_closed = self.setToolControl(joint_angle)




        
                
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
                    path_time=0.8
                    print("PEQUEÑO MI X. esta a la izquierda del centroide, MOVER A LA DERECHA")
                    print(f"OBJETIVO: {kinematics_pose}")
                    print(f"{x,y}")
                    aka=self.setTaskSpacePath(kinematics_pose,path_time)
                    rospy.sleep(1)  
                    print(f"CENTROIDE:{x,y}")
                    if y < y_min: #Y ES GRANDE
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]+0.01, position[1], position[2]]
                        path_time=0.8
                        print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                        print(f"OBJETIVO: {kinematics_pose}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(1)
                        print(f"CENTROIDE:{x,y}")
                    if y > y_max: #Y ES PEQUEÑO
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]-0.01, position[1], position[2]]
                        path_time=0.8
                        print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                        print(f"OBJETIVO: {kinematics_pose}")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(1)
                        print(f"CENTROIDE:{x,y}")
                
                if x > x_max: #X GRANDE
                    position = self.getPresentKinematicsPose()
                    print(f" POSITION ACTUAL {position}")
                    kinematics_pose= [position[0], position[1]-0.01, position[2]]
                    path_time=1
                    print("PEQUEÑO MI X. esta a la derecha del centroide, MOVER A LA IZQUIERDA")
                    print(f"OBJETIVO: {kinematics_pose}")
                    aka=self.setTaskSpacePath(kinematics_pose,path_time)
                    rospy.sleep(1)
                    print(f"CENTROIDE:{x,y}")
                    if y < y_min: #Y ES GRANDE
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]+0.01, position[1], position[2]]
                        path_time=2.0
                        print("GRANDE MI Y. esta arriba del centroide, MOVER ABAJO")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(1)
                        print(f"CENTROIDE:{x,y}")
                    if y > y_max: #Y ES PEQUEÑO
                        position = self.getPresentKinematicsPose()
                        print(f" POSITION ACTUAL {position}")
                        kinematics_pose= [position[0]-0.01, position[1], position[2]]
                        path_time=0.8
                        print("PEQUEÑO MI Y. esta abajo del centroide, MOVER ARRIBA")
                        aka=self.setTaskSpacePath(kinematics_pose,path_time) 
                        rospy.sleep(1)
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

    def Tool_attach(self,joint_angle): 
        if joint_angle == [-0.01]:
            #while detectedObject is None:
            self.attach_detect()
        if joint_angle == [0.01]:
            #while detectedObject is None:
            self.detach_detect()
        else:
            print("no señal")

    def attach_detect(self):
        if self.detectedObject is not None and self.resetvalue is not True:
            rospy.loginfo(f"Attaching gripper and {self.detectedObject}")
            req = AttachRequest()
            req.model_name_1 = "open_manipulator"
            req.link_name_1 = "gripper_link_sub"
            req.model_name_2 = self.detectedObject
            req.link_name_2 = "link"
            self.attach_srv.call(req)


        else:
            print("objeto no detectado -> no collision")

    def detach_detect(self): #hay problema si no agarra nada
        if self.detectedObject is not None:
            rospy.loginfo(f"Detach gripper and {self.detectedObject}")
            req1 = AttachRequest()
            req1.model_name_1 = "open_manipulator"
            req1.link_name_1 = "gripper_link_sub"
            req1.model_name_2 = self.detectedObject
            req1.link_name_2 = "link"
            self.detach_srv.call(req1)
            

    def get_contacts(self, msg):
        if (len(msg.states) == 0):
            self.resetvalue=True
            #rospy.loginfo("No contacts were detected!")
        else:
            if 'gripper_link_sub' in msg.states[0].collision1_name:
                self.resetvalue=False
                #rospy.loginfo("Collision 1 detected with %s." % msg.states[0].collision2_name.split("::")[0])
                objeto = msg.states[0].collision2_name.split("::")[0]
                objetos_validos = ["esfera_roja", "esfera_azul", "esfera_verde","cubo_rojo", "cubo_verde", "cubo_azul","cubo_verdep", "esfera_rojap"]
                #print(f"{objeto}")
                if objeto in objetos_validos:
                    self.detectedObject=objeto
                    

                    
                    
            elif 'gripper_link_sub' in msg.states[0].collision2_name:
                self.resetvalue=False
                #rospy.loginfo("Collision 2 detected with %s." % msg.states[0].collision1_name.split("::")[0])
                objeto = msg.states[0].collision1_name.split("::")[0]
                #print(f"{objeto}")
                objetos_validos = ["esfera_roja", "esfera_azul", "esfera_verde","cubo_rojo", "cubo_verde", "cubo_azul","cubo_verdep", "esfera_rojap"]
                if objeto in objetos_validos:
                    self.detectedObject=objeto
    def run(self):
        """ Mantiene el nodo ROS en ejecución. """
        rospy.spin()
        
if __name__ == "__main__":
    try:
        nodo = process()
        nodo.run()
    except rospy.ROSInterruptException:
        pass