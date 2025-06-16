#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2  # OpenCV
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String
import time
import tty
import termios
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


class pluggin_attach:

    def __init__(self):
        self.kinematics_pose = None
        self.present_kinematic_position = [0.0, 0.0, 0.0]  # Inicialización segura
        self.open_manipulator_is_moving = False
        self.open_manipulator_actuator_enabled = False
        
        rospy.init_node('open_manipulator_teleop_keyboard', anonymous=True)
        self.namespace = rospy.get_param('~namespace', 'default_value') 
        self.orden_input=None
        self.joint_state_subscriber = rospy.Subscriber(f'{self.namespace}/joint_states', JointState, self.joint_states_callback)
        self.kinematics_pose_subscriber = rospy.Subscriber(f'{self.namespace}/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        self.open_manipulator_states_sub = rospy.Subscriber(f"{self.namespace}/states", OpenManipulatorState, self.manipulator_states_callback)
        self.publicador_centroide = rospy.Publisher(f'{self.namespace}/objeto/Centroide', Point, queue_size=10)
        self.publicador_dimensiones= rospy.Publisher(f'{self.namespace}/dimensiones_imagen', Point, queue_size=10)
        self.sub_contacts = rospy.Subscriber (f'{self.namespace}/contact_states', ContactsState, self.get_contacts)

        self.set_actuator_state_client = rospy.ServiceProxy(f'{self.namespace}/set_actuator_state', SetActuatorState)
        self.set_joint_position_client = rospy.ServiceProxy(f'{self.namespace}/goal_joint_space_path', SetJointPosition)
        self.set_tool_control_client = rospy.ServiceProxy(f'{self.namespace}/goal_tool_control', SetJointPosition)
        self.goal_task_space_path_position_only_client = rospy.ServiceProxy('/goal_task_space_path_position_only', SetKinematicsPose)
        self.detectedObject = None
        self.detach_srv = rospy.ServiceProxy(f'{self.namespace}/link_attacher_node/detach',Attach)
        self.attach_srv = rospy.ServiceProxy(f'{self.namespace}/link_attacher_node/attach',Attach)
        self.habilitar_actuadores()
        self.fuction_imprimir_menu()

    def habilitar_actuadores(self):
        self.setActuatorState(True)  # Llamada para habilitar los actuadores

        
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
        
    def Tool_attach(self,joint_angle): 
        if joint_angle==[-0.01]:
            #while detectedObject is None:
            self.attach_detect()
        if joint_angle== [0.01]:
            #while detectedObject is None:
            self.detach_detect()


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


    def leer_tecla(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)           # Modo sin buffer: captura tecla inmediatamente
            ch = sys.stdin.read(1)   # Lee 1 carácter
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def fuction_imprimir_menu(self):            
        comandos_validos = {'w', 's', 'a', 'd', 'z', 'x',
                            'y', 'h', 'u', 'j', 'i', 'k', 'o', 'l',
                            'g', 'f', '1', '2'}

        while not rospy.is_shutdown():
            print("---------------------------------------------------")
            print("LISTO PARA CONTROLAR TU OPENMANIPULATOR EN GAZEBO---------------------------")
            print("     w : incrementa eje X en el espacio de tareas")
            print("     s : decrementa eje X en el espacio de tareas")
            print("     a : incrementa eje Y en el espacio de tareas")
            print("     d : decrementa eje Y en el espacio de tareas")
            print("     z : incrementa eje Z en el espacio de tareas")
            print("     x : decrementa eje Z en el espacio de tareas\n\n")

            print("     y : Aumentar el ángulo de la joint 1")
            print("     h : Disminuye el ángulo de la joint 1")
            print("     u : Aumentar el ángulo de la joint 2")
            print("     j : Disminuye el ángulo de la joint 2")
            print("     i : Aumentar el ángulo de la joint 3")
            print("     k : Disminuye el ángulo de la joint 3")
            print("     o : Aumentar el ángulo de la joint 4")
            print("     l : Disminuye el ángulo de la joint 4\n\n")

            print("     g : Abrir gripper")
            print("     f : Cerrar gripper")
                    
            print("     1 : Posición inicial")
            print("     2 : Posición home\n\n")
            print("     q : Salir del nodo")
            print("---------------------------------------------------")

            print("Entrada:")
            respuesta = self.leer_tecla()
            respuesta = respuesta.strip().lower()

            if respuesta == 'q':
                rospy.signal_shutdown("Se solicitó terminar el nodo.")
                break

            elif respuesta in comandos_validos:
                if(respuesta=='w'):
                    print("   w:   INCREMENTANDO ++ EJE X")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                    self.incrementar_x()
                if(respuesta=='s'):
                    self.decrementar_x()
                    print("   s:   DECREMENTANDO -- EJE X")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='a'):
                    self.incrementar_y()
                    print("   a:   INCREMENTANDO ++ EJE Y")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='d'):
                    self.decrementar_y()
                    print("   d:   DECREMENTANDO -- EJE Y")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='z'):        
                    self.incrementar_z()
                    print("   z:   INCREMENTANDO ++ EJE Z")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='x'):        
                    self.decrementar_z()
                    print("   x:   DECREMENTANDO -- EJE Z")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='y'):        
                    self.aumentar_j1()
                    print("   y:   AUMENTANDO ++ J1")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='h'):
                    self.disminuir_j1()
                    print("   h:   DISMINUYENDO -- J1")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='u'):        
                    self.aumentar_j2()
                    print("   u:   AUMENTANDO ++ J2")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='j'):        
                    self.disminuir_j2()
                    print("   j:   DISMINUYENDO -- J2")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='i'):        
                    self.aumentar_j3()
                    print("   i:   AUMENTANDO ++ J3")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='k'):        
                    self.disminuir_j3()
                    print("   k:   DISMINUYENDO -- J3")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='o'):        
                    self.aumentar_j4()
                    print("   o:   AUMENTANDO ++ J4")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='l'):        
                    self.disminuir_j4()
                    print("   l:   DISMINUYENDO -- J4")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='g'):        
                    self.abrir_gripper()
                    print("   g:   ABRIENDO GRIPPER")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='f'):        
                    self.cerrar_gripper()
                    print("   f:   CERRANDO GRIPPER")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='1'):        
                    self.posicion_inicial()
                    print("   1:   DIRIGIENDO A POSICION INICIAL")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                if(respuesta=='2'):        
                    self.posicion_home()
                    print("   2:   DIRIGIENDO A POSICION HOME")
                    pos_kinematics = self.getPresentKinematicsPose()
                    pos_articular= self.getPresentJointAngle()
                    print(f"Estado actual del Robot en espacio articular: j1={pos_articular[0]}, j2={pos_articular[1]}, j3={pos_articular[2]}, j4={pos_articular[3]}")
                    print(f"Estado actual del Robot en plano cartesiano: X={pos_kinematics[0]}, Y={pos_kinematics[1]}, Z={pos_kinematics[2]}")
                    print(f"Estado actual del gripper: Gripper={pos_articular[4]}")
                    
            else:
                print("Entrada no válida. Intenta nuevamente.")

    def incrementar_x(self):
        position = self.getPresentKinematicsPose()
        kinematics_pose= [position[0]+0.01, position[1], position[2]]
        path_time=0.5
        pos=self.setTaskSpacePath(kinematics_pose,path_time)
        
    def decrementar_x(self):
        position = self.getPresentKinematicsPose()
        kinematics_pose= [position[0]-0.01, position[1], position[2]]
        path_time=0.5
        pos=self.setTaskSpacePath(kinematics_pose,path_time)
        
    def incrementar_y(self):
        position = self.getPresentKinematicsPose()
        kinematics_pose= [position[0], position[1]+0.01, position[2]]
        path_time=0.5
        pos=self.setTaskSpacePath(kinematics_pose,path_time)
        
    def decrementar_y(self):
        position = self.getPresentKinematicsPose()
        kinematics_pose= [position[0], position[1]-0.01, position[2]]
        path_time=0.5
        pos=self.setTaskSpacePath(kinematics_pose,path_time)
        
    def decrementar_z(self):
        position = self.getPresentKinematicsPose()
        kinematics_pose= [position[0], position[1], position[2]-0.01]
        path_time=0.5
        pos=self.setTaskSpacePath(kinematics_pose,path_time)

    def incrementar_z(self):
        position = self.getPresentKinematicsPose()
        kinematics_pose= [position[0], position[1], position[2]+0.01]
        path_time=0.5
        pos=self.setTaskSpacePath(kinematics_pose,path_time)

    def aumentar_j1(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0]+0.05, pos_articular[1], pos_articular[2], pos_articular[3]]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def disminuir_j1(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0]-0.05, pos_articular[1], pos_articular[2], pos_articular[3]]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def aumentar_j2(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0], pos_articular[1]+0.05, pos_articular[2], pos_articular[3]]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def disminuir_j2(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0], pos_articular[1]-0.05, pos_articular[2], pos_articular[3]]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def aumentar_j3(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0], pos_articular[1], pos_articular[2]+0.05, pos_articular[3]]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def disminuir_j3(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0], pos_articular[1], pos_articular[2]-0.05, pos_articular[3]]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)
    def aumentar_j4(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0], pos_articular[1], pos_articular[2], pos_articular[3]+0.05]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def disminuir_j4(self):
        pos_articular= self.getPresentJointAngle()
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle=[pos_articular[0], pos_articular[1], pos_articular[2], pos_articular[3]-0.05]
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)


    def abrir_gripper(self):
        joint_angle=[0.01]
        gripper_open = self.setToolControl(joint_angle)
        kaka= self.Tool_attach(joint_angle)

    def cerrar_gripper(self):
        joint_angle=[-0.01]
        gripper_closed = self.setToolControl(joint_angle)
        kaka2= self.Tool_attach(joint_angle)

    def posicion_inicial(self):
        joint_angle = 0.01
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle = [0.0, -1.05, 0.35, 0.70]  # Ángulos de la posición inicial
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)

    def posicion_home(self):
        joint_angle = 0.01
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        joint_angle = [0.000, 0.000 , 0.000, 0.000]  # Ángulos de la posición inicial
        path_time = 0.5  # Tiempo de movimiento
        success = self.setJointSpacePath(joint_name, joint_angle, path_time)



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
        rospy.spin()        

if __name__ == "__main__":
    try:
        nodo = pluggin_attach()
        nodo.run()
    except rospy.ROSInterruptException:
        pass